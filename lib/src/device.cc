#include <l4/re/error_helper>
#include <l4/re/env>

#include <l4/vbus/vbus>

#include <l4/ixylon/device.h>

#include "driver/e1000/e1000.h"
#include "driver/ixgbe/ixgbe.h"
#include "pci.h"

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

/* Create and initialize the driver for a certain device.                   */
Ixl_device* Ixl_device::ixl_init(L4::Cap<L4vbus::Vbus> vbus,
                                 uint32_t dev_idx, uint16_t rx_queues,
                                 uint16_t tx_queues, int irq_timeout) {
    // Read PCI configuration space
    // For VFIO, we could access the config space another way
    // (VFIO_PCI_CONFIG_REGION_INDEX). This is not needed, though, because
    // every config file should be world-readable, and here we
    // only read the vendor and device id.
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
    switch (vendor_id) {
        // Intel
        case 0x8086:
            switch (device_id) {
                case E1000_DEV_ID_82540EM:
                    // The device emulated by QEMU when choosing an e1000 NIC
                    ixl_info("Trying e1000...");
                    return E1000_device::e1000_init(std::move(dev),
                                                    rx_queues, tx_queues,
                                                    irq_timeout);
                    break;
                case IXGBE_DEV_ID_82598:
                    ixl_info("Trying ixgbe...");
                    return Ixgbe_device::ixgbe_init(std::move(dev),
                                                    rx_queues, tx_queues,
                                                    irq_timeout);
                    break;
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
}
