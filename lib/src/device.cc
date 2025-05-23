/****************************************************************************
 *
 * Implementation of the common functions of an Ixl_device.
 *
 * Copyright (C) 2023 - 2025 Till Miemietz
 *                           <till.miemietz@barkhauseninstitut.org>
 */


/****************************************************************************
 *                                                                          *
 *                           include statements                             *
 *                                                                          *
 ****************************************************************************/


#include <l4/re/error_helper>
#include <l4/re/env>
#include <pthread-l4.h>

#include <l4/vbus/vbus>

#include <l4/ixl/device.h>

#include "driver/e1000/e1000.h"
#include "driver/igb/igb.h"
#include "driver/ixgbe/ixgbe.h"
#include "pci.h"

/****************************************************************************
 *                                                                          *
 *                        function implementation                           *
 *                                                                          *
 ****************************************************************************/


using namespace Ixl;

/* Create a DMA space for this device.                                      */
void Ixl_device::create_dma_space(void) {
    // Obtain information about the device (e.g. no. of resources & their types)
    l4vbus_device_t devinfo;
    L4Re::chksys(pci_dev.device(&devinfo));

    ixl_debug("dev has %u resources", devinfo.num_resources);

    // Iterate through list of device resoures to find the DMA space
    unsigned int i;
    l4vbus_resource_t res;
    for (i = 0; i < devinfo.num_resources; i++) {
        L4Re::chksys(pci_dev.get_resource(i, &res));

        if (res.type == L4VBUS_RESOURCE_DMA_DOMAIN)
            break;

        ixl_debug("Skipping resource of type %u...", res.type);
    }

    if (i == devinfo.num_resources)
        ixl_error("Did not find DMA domain. Aborting...");

    dma_cap = L4Re::chkcap(L4Re::Util::make_shared_cap<L4Re::Dma_space>(),
                           "Failed to allocate capability slot.");

    L4Re::chksys(L4Re::Env::env()->user_factory()->create(dma_cap.get()),
                 "Failed to create DMA space.");

    // This is the step where we bind the DMA domain of the device to the
    // DMA space we just created for this task. The first argument to the
    // assignment operation is the domain_id, i.e. the starting address of
    // the DMA domain found in the set of device resources.
    // TODO: We could also choose the DMA domain of the whole vbus with ~0x0...
    L4Re::chksys(pci_dev.bus_cap()->assign_dma_domain(res.start,
                        L4VBUS_DMAD_BIND | L4VBUS_DMAD_L4RE_DMA_SPACE,
                        dma_cap.get()),
                        "Failed to bind to device's DMA domain.");

    ixl_debug("Created DMA space for device");
}

/* Retrieves the vICU cap of the vbus that pci_dev is located on            */
void Ixl_device::setup_icu_cap(void) {
    L4vbus::Icu   icudev;           // vICU of device's vbus
    l4_icu_info_t icu_info;         // Struct with some information on the vICU

    auto vbus = pci_dev.bus_cap();

    // For each vbus, there is an ICU and it has a fixed hardware ID
    if (vbus->root().device_by_hid(&icudev, "L40009") < 0)
        ixl_error("Failed to get ICU device.");

    interrupts.vicu = L4Re::chkcap(L4Re::Util::cap_alloc.alloc<L4::Icu>(),
                                   "Failed to allocate ICU capability.");

    if (icudev.vicu(interrupts.vicu) != 0)
        ixl_error("Failed to request ICU capability.");

    // Do some sanity checks. If there is MSI(X) support, we want to have
    // a separate IRQ per receive queue.
    if (l4_error(interrupts.vicu->info(&icu_info)) < 0)
        ixl_error("Failed to get ICU info");

    ixl_debug("ICU info: features = %x, #IRQs = %u, #MSIs = %u",
              icu_info.features, icu_info.nr_irqs, icu_info.nr_msis);

    if (icu_info.features & L4::Icu::F_msi) {
        if (icu_info.nr_msis < num_rx_queues) {
            ixl_error("ICU supports only %u MSIs, but driver needs %u.",
                      icu_info.nr_msis, num_rx_queues);
        }
    }
}

/* Get IRQ for the given RX queue.                                          */
L4::Cap<L4::Irq> Ixl_device::get_recv_irq(uint16_t qid) {
    if (qid >= num_rx_queues) {
        ixl_warn("Invalid qid paramter (out of bounds).");
        return L4::Cap<L4::Irq>();
    }

    if (interrupts.mode == interrupt_mode::Disable) {
        ixl_info("Ignoring request as IRQs are disabled globally.");
        return L4::Cap<L4::Irq>();
    }

    return interrupts.queues[qid].irq;
}

/* Rebind the IRQ of a receive queue to the calling thread.                 */
void Ixl_device::rebind_recv_irq(uint16_t qid) {
    if (qid >= num_rx_queues) {
        ixl_warn("Invalid qid paramter (out of bounds).");
        return;
    }

    if (interrupts.mode == interrupt_mode::Disable) {
        ixl_info("Ignoring request as IRQs are disabled globally.");
        return;
    }

    L4::Cap<L4::Irq> irq = interrupts.queues[qid].irq;
    L4Re::chksys(l4_error(irq->bind_thread(Pthread::L4::cap(pthread_self()), 0)),
                 "Failed to bind IRQ to calling thread.");
}

/* Create and initialize the driver for a certain device.                   */
Ixl_device* Ixl_device::ixl_init(L4::Cap<L4vbus::Vbus> vbus,
                                 uint32_t dev_idx, struct Dev_cfg &cfg) {
    // Read PCI configuration space to obtain initial device information.
    uint32_t vendor_id;
    uint32_t device_id;

    // We search for Ethernet devices only (class 0x2, subclass 0x0)
    L4vbus::Pci_dev dev = pci_get_dev(vbus, dev_idx, 0x02, 0x00);
    check_err(dev.cfg_read(0, &vendor_id, 16), "Failed to read PCI vendor ID");
    check_err(dev.cfg_read(2, &device_id, 16), "Failed to read PCI device ID");

    // Choose driver according to vendor / device ID combination
    // For now, we will exclude all unknown devices by default, even though a
    // particular driver might work. With more time and testing, we can add
    // more supported devices.
    ixl_debug("Device vendor ID = 0x%x, device ID = 0x%x", vendor_id,
              device_id);
    switch (vendor_id) {
        // Intel
        case 0x8086:
            switch (device_id) {
                // E1000-compatible devices
                case E1000_DEV_ID_82540EM:
                    // The device emulated by QEMU when choosing an e1000 NIC
                    ixl_info("Trying e1000...");
                    return E1000_device::e1000_init(std::move(dev), cfg);
                // Hereinafter all igb-driver devices
                case IGB_DEV_ID_82576:
                case IGB_DEV_ID_I350:
                    ixl_warn("The Igb driver provides only a limited feature "
                             "set. You have been warned!");
                    return Igb_device::igb_init(std::move(dev), cfg);
                // Hereinafter all ixgbe-driven devices
                case IXGBE_DEV_ID_X540T:
                case IXGBE_DEV_ID_82598:
                    ixl_info("Trying ixgbe...");
                    return Ixgbe_device::ixgbe_init(std::move(dev), cfg);
                default:
                    ixl_error("Unsupported device %x of vendor %x. "
                              "No suitable driver found.", device_id,
                              vendor_id);
                break;
            }
            break;
        default:
            ixl_error("Unknown vendor %x. No suitable driver found.",
                      vendor_id);
            break;
    }

    return NULL;
}
