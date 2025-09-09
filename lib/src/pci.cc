/****************************************************************************
 *
 * Implementation of PCI-related functions on L4Re.
 *
 * Copyright (C) 2023 - 2025 Till Miemietz
 *                           <till.miemietz@barkhauseninstitut.org>
 */


/****************************************************************************
 *                                                                          *
 *                           include statements                             *
 *                                                                          *
 ****************************************************************************/


#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include <l4/re/env>
#include <l4/re/error_helper>
#include <l4/vbus/vbus>
#include <l4/vbus/vbus_pci>
#include <l4/vbus/vbus_interfaces.h>

#include <l4/ixl/log.h>

#include "pci.h"

using namespace Ixl;

/****************************************************************************
 *                                                                          *
 *                        static helper functions                           *
 *                                                                          *
 ****************************************************************************/


/**
 * Checks wether a PCI device has a certain capability present in its
 * configuration space. If the capability is present and addr is not NULL,
 * the address of the searched capability (i.e., its offset in the config
 * space) is returned through addr.
 *
 * \param dev    The PCI device to check.
 * \param cap_id The ID of the capability to check for.
 * \param addr   The offset of the capability in the config space.
 *
 * \returns True if the capability is present, false otherwise.
 */
static bool check_pci_cap(L4vbus::Pci_dev &dev, uint8_t cap_id, uint32_t *addr){
    uint32_t config;                // Content of PCI config register of dev

    // Get the config register (offset 0x06 into the config space)
    L4Re::chksys(dev.cfg_read(0x06, &config, 16));
    ixl_debug("Checking for PCI cap %u", cap_id);

    // Check if the device has a capability list (Bit 4 is set)
    if (config & 0x10) {
        uint32_t cap_ptr;           // Pointer to next capability entry
                                    // (offset in PCI config space)

        // The first cap entry is at offset 0x34
        L4Re::chksys(dev.cfg_read(0x34, &cap_ptr, 8));
        // Mask off the two least significant bits as they are reserved
        cap_ptr = cap_ptr & ~0x03;

        while (cap_ptr != 0) {
            uint32_t cap_id32 = 0;

            L4Re::chksys(dev.cfg_read(cap_ptr, &cap_id32, 8));

            // Cap ID for MSI-X is 0x11
            if (cap_id32 == cap_id) {
                if (addr != NULL)
                    *addr = cap_ptr;

                return true;
            }

            // The next cap offset is one Byte behind the ID
            L4Re::chksys(dev.cfg_read(cap_ptr + 1, &cap_ptr, 8));
            cap_ptr = cap_ptr & ~0x03;
        }
    }

    return false;
}

/****************************************************************************
 *                                                                          *
 *                        function implementation                           *
 *                                                                          *
 ****************************************************************************/


/* Compute the size of the I/O memory accessible through BAR<idx>           */
l4_uint64_t Ixl::get_bar_size(L4vbus::Pci_dev& dev, unsigned int idx) {
    // TODO: According to OSdev wiki, one should also disable the I/O and
    //       memory decode bytes in the command register before executing
    //       this operation...
    l4_uint64_t bar_size = 0;

    l4_uint32_t lsb = ~((l4_uint32_t) 0x0);
    l4_uint32_t msb = ~((l4_uint32_t) 0x0);
    l4_uint32_t lsb_old;
    l4_uint32_t msb_old;

    // Save original content of BAR
    L4Re::chksys(dev.cfg_read(0x10 + idx * 4, &lsb_old, 32));
    L4Re::chksys(dev.cfg_read(0x14 + idx * 4, &msb_old, 32));

    // Check if the device is memory but 32-bit addressed (Bit 2 != 1))
    if (! (lsb_old & 0x00000004)) {
        // Write all-ones to the register and read back the BAR value
        L4Re::chksys(dev.cfg_write(0x10 + idx * 4, lsb, 32));
        L4Re::chksys(dev.cfg_read(0x10 + idx * 4, &lsb, 32));

        // restore the original register contents
        L4Re::chksys(dev.cfg_write(0x10 + idx * 4, lsb_old, 32));

        // Now compute the size of the BAR memory. Remember to clear the lower
        // four bits of the LSB register part
        lsb      = lsb & 0xfffffff0;
        bar_size = (l4_uint64_t) ~lsb;
    }
    else {
        // Device is 64-bit addressed, so the MSB part of the BAR size is
        // contained in the next MSB.

        // Write all-ones to the register and read back the BAR value
        L4Re::chksys(dev.cfg_write(0x10 + idx * 4, lsb, 32));
        L4Re::chksys(dev.cfg_write(0x14 + idx * 4, msb, 32));
        L4Re::chksys(dev.cfg_read(0x10 + idx * 4, &lsb, 32));
        L4Re::chksys(dev.cfg_read(0x14 + idx * 4, &msb, 32));

        // restore the original register contents
        L4Re::chksys(dev.cfg_write(0x10 + idx * 4, lsb_old, 32));
        L4Re::chksys(dev.cfg_write(0x14 + idx * 4, msb_old, 32));

        // Now compute the size of the BAR memory. Remember to clear the lower
        // four bits of the LSB register part
        lsb      = lsb & 0xfffffff0;
        bar_size = ((l4_uint64_t) msb << 32) | lsb;
        bar_size = ~bar_size;
    }

    return bar_size + 1;
}

/* Get the physical address of BAR<idx>                                     */
l4_uint64_t Ixl::get_bar_addr(L4vbus::Pci_dev& dev, unsigned int idx) {
    l4_uint32_t lsb;
    l4_uint32_t msb;

    L4Re::chksys(dev.cfg_read(0x10 + idx * 4, &lsb, 32));

    // Always mask the lower bits of the LSB (mapping is page-aligned)

    // I/O Spaces are always 32-bit addressed
    if (lsb & 0x00000001)
        return ((l4_uint64_t) lsb) & 0xfffffffffffff000UL;

    // Check if the device is memory but 32-bit addressed (Bit 2 != 1))
    if (! (lsb & 0x00000004))
        return ((l4_uint64_t) lsb) & 0xfffffffffffff000UL;

    // Device is 64-bit addressed, read the next BAR to obtain the MSB part of
    // the address
    L4Re::chksys(dev.cfg_read(0x14 + idx * 4, &msb, 32));

    return (((l4_uint64_t) msb) << 32 | lsb) & 0xfffffffffffff000UL;
}

void Ixl::enable_dma(L4vbus::Pci_dev& dev) {
    l4_uint32_t cmd_reg = 0;                  // Value of PCI command register

    // write to the command register (offset 4) in the PCIe config space
    // bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
    L4Re::chksys(dev.cfg_read(0x4, &cmd_reg, 16));
    cmd_reg |= 1 << 2;
    L4Re::chksys(dev.cfg_write(0x4, cmd_reg, 16));
}

/* Map a BAR to the address space of the executing task.                    */
uint8_t* Ixl::pci_map_bar(L4vbus::Pci_dev& dev, unsigned int idx) {
    l4_addr_t   iomem_addr;       // Address for mapping the I/O memory of BAR0
    l4_uint64_t iomem_size;       // Size of memory accessible through BAR0
    l4_uint64_t bar_addr;         // Physical address contained in BAR0

    enable_dma(dev);

    // Obtain information about the device (e.g. no. of resources & their types)
    // This is only for debugging purposes, though.
    l4vbus_device_t devinfo;
    L4Re::chksys(dev.device(&devinfo));

    ixl_debug("Dev has %u resources", devinfo.num_resources);

    iomem_size = get_bar_size(dev, idx);
    bar_addr   = get_bar_addr(dev, idx);

    ixl_debug("BAR%u addr = %llx, BAR%u size = %llx", idx, bar_addr,
              idx, iomem_size);

    L4::Cap<L4Re::Dataspace> ds =
            L4::cap_reinterpret_cast<L4Re::Dataspace>(dev.bus_cap());

    // Ok, so the Vbus of the device appears to also act as a dataspace that
    // allows for attaching all I/O memory resources of a device. We'll use
    // that to map the I/O memory of the PCI device (which is described in
    // the BARs).
    //
    // The Vbus spans the entire address space of the machine (although we can
    // only map the memory that belongs to the devices attached to the vbus).
    // Hence, we use the physical address contained inside the BAR as an offset
    // into the special Vbus dataspace.
    L4Re::chksys(L4Re::Env::env()->rm()->attach(
        &iomem_addr, l4_round_page(iomem_size),
        L4Re::Rm::F::Search_addr | L4Re::Rm::F::Cache_uncached | L4Re::Rm::F::RW,
        L4::Ipc::make_cap_rw(ds), bar_addr, L4_PAGESHIFT));

    ixl_debug("Mapped BAR%u to address %lx", idx, iomem_addr);
    return (uint8_t *) iomem_addr;
}

/* Check if the device supports MSI-X                                       */
bool Ixl::pcidev_supports_msix(L4vbus::Pci_dev &dev) {
    return check_pci_cap(dev, 0x11, NULL);
}

/* Check if the device supports MSI                                         */
bool Ixl::pcidev_supports_msi(L4vbus::Pci_dev &dev) {
    return check_pci_cap(dev, 0x05, NULL);
}

/* Performs the initial setup of MSIs on a PCI device                       */
void Ixl::setup_msi(L4vbus::Pci_dev &dev) {
    uint32_t msi_cap_addr;              // Address of MSI capability
    uint32_t msi_ctrl_reg;              // Contents of MSI control register
    uint32_t mmc;                       // Multi-message capability of MSI

    // Get the MSI capability address through a feature check
    if (! check_pci_cap(dev, 0x05, &msi_cap_addr))
        return;

    L4Re::chksys(dev.cfg_read(msi_cap_addr + 2, &msi_ctrl_reg, 16));
    mmc = msi_cap_addr & 0x0000000e;

    // Set MME (multi-message enabled) to the same value as the MMC, i.e.,
    // allocate the maximum number of MSIs for this device (clear MME first).
    msi_ctrl_reg &= 0xffffff8f;
    msi_ctrl_reg |= mmc << 3;

    // Enable MSIs
    msi_ctrl_reg |= 0x01;

    // Write back the MSI configuration
    L4Re::chksys(dev.cfg_write(msi_cap_addr + 2, msi_ctrl_reg, 16));
}

/* Enables the MSI no. irqnum on the PCI level.                             */
void Ixl::pcidev_enable_msi(L4vbus::Pci_dev &dev, uint32_t,
                            l4_icu_msi_info_t info) {
    uint32_t msi_cap_addr;              // Address of MSI capability
    uint32_t msi_ctrl_reg;              // Contents of MSI control register
    uint32_t mmc;                       // Multi-message capability of MSI

    // Get the MSI capability address through a feature check
    if (! check_pci_cap(dev, 0x05, &msi_cap_addr))
        return;

    L4Re::chksys(dev.cfg_read(msi_cap_addr + 2, &msi_ctrl_reg, 16));
    mmc = msi_cap_addr & 0x0000000e;

    // We write the address as given by the info structure, and just clear the
    // least significant bits according to the maximum number of supported MSIs.
    L4Re::chksys(dev.cfg_write(msi_cap_addr + 4,
                               info.msi_addr & 0xffffffff, 32));

    // Check if the MSI register it 64 bit and proceed accordingly (bit 7)
    if (msi_ctrl_reg & 0x00000080) {
        L4Re::chksys(dev.cfg_write(msi_cap_addr + 8,
                                   (uint32_t) (info.msi_addr >> 32), 32));
        L4Re::chksys(dev.cfg_write(msi_cap_addr + 0xc,
                                   info.msi_data & ~(mmc - 1), 16));
    }
    else {
        L4Re::chksys(dev.cfg_write(msi_cap_addr + 8,
                                   info.msi_data & ~(mmc - 1), 16));
    }
}

/* Performs the initial setup of MSI-Xs on a PCI device                     */
void Ixl::setup_msix(L4vbus::Pci_dev &dev) {
    uint32_t msix_cap_addr;             // Address of MSI-X capability
    uint32_t msix_ctrl_reg;             // Contents of MSI-X control register

    // Get the MSI-X capability address through a feature check
    if (! check_pci_cap(dev, 0x11, &msix_cap_addr))
        return;

    L4Re::chksys(dev.cfg_read(msix_cap_addr + 2, &msix_ctrl_reg, 16));

    // Set the enabled bit of the ctrl register
    msix_ctrl_reg |= 0x00008000;
    // Clear the function mask bit of the ctrl register
    msix_ctrl_reg &= 0xffffbfff;

    // Write back the MSI-X configuration
    L4Re::chksys(dev.cfg_write(msix_cap_addr + 2, msix_ctrl_reg, 16));
}

/* Retrieves information about the MSI-X table of a PCI device.             */
void Ixl::pcidev_get_msix_info(L4vbus::Pci_dev &dev, uint32_t *bir,
                               uint32_t *table_offs, uint32_t *table_size) {
    uint32_t msix_cap_addr;         // Address of MSI-X capability
    uint32_t msix_ctrl_reg;         // Contents of MSI-X control register

    // Get the MSI-X capability address through a feature check
    if (! check_pci_cap(dev, 0x11, &msix_cap_addr))
        return;

    // Get register contents.
    L4Re::chksys(dev.cfg_read(msix_cap_addr + 2, &msix_ctrl_reg, 16));
    *table_size = msix_ctrl_reg & 0x000007ff;        // Size is bits 0 - 10

    L4Re::chksys(dev.cfg_read(msix_cap_addr + 4, table_offs, 32));
    *bir = *table_offs & 0x7;               // Lowest three bits make up BIR
    *table_offs &= ~((uint32_t) 0x07);      // Offset is 8-Byte aligned
}

/* Enables the MSI-X no. irqnum on the PCI level.                           */
void Ixl::pcidev_enable_msix(uint32_t irqnum, l4_icu_msi_info_t info,
                             uint8_t *bar_addr, uint32_t table_offs,
                             uint32_t table_size) {
    // Start address of table entry to modify (each entry is 16 B in size)
    uint32_t reg = table_offs + irqnum * 16;

    // Table size is (N - 1) encoded
    if (irqnum > table_size) {
        ixl_warn("Refusing to write out-of-bounds MSI-X table entry!");
        return;
    }

    // Write the MSI-X table for the specified IRQ (addr is four-Byte aligned)
    set_reg32(bar_addr, reg + 0, info.msi_addr & 0xfffffffc);
    set_reg32(bar_addr, reg + 4, info.msi_addr >> 32);
    set_reg32(bar_addr, reg + 8, info.msi_data);
    // Unmask this MSI-X
    clear_flags32(bar_addr, reg + 12, 0x00000001);
}

/* Acquires a PCI devices at a certain index on the "vbus" capability.      */
L4vbus::Pci_dev Ixl::pci_get_dev(L4::Cap<L4vbus::Vbus> vbus,
                                 uint32_t idx, uint8_t pci_class,
                                 uint8_t pci_sclass)
{
    ixl_debug("Starting device discovery...");
    auto root = vbus->root();
    L4vbus::Pci_dev child{};            // Child device for iterating
    l4vbus_device_t dev_info;           // L4vbus device info about child

    unsigned int i         = 0;
    uint8_t      class_id  = 0;         // PCI device class ID
    uint8_t      sclass_id = 0;         // PCI device subclass ID

    // Iterate the vbus until we found the device at index dev_idx
    while (root.next_device(&child, L4VBUS_MAX_DEPTH, &dev_info) == L4_EOK) {
        uint32_t pci_id_reg = 0;

        // Even when only matching for PCI devices, the vbus may contain other
        // devices such as a device for the ICU. We should not get confused by
        // those. Non-PCI devices must neither influence matching of PCI
        // devices by index on the vbus.
        if (! l4vbus_subinterface_supported(dev_info.type,
                                            L4VBUS_INTERFACE_PCIDEV)) {
            ixl_debug("Skipping non-PCI device of type %x...", dev_info.type);
            continue;
        }

        // Make sure that the device found matches the desired dev class
        L4Re::chksys(child.cfg_read(8, &pci_id_reg, 32),
                     "Failed to read PCI class ID");

        // We just read the third register of the device's configuration
        // space, which is subdivided as follows:
        //
        // | Bit 31 - 24  | Bit 23 - 16 | Bit 15 - 8            | Bit 7 - 0   |
        // | Dev class ID | Subclass ID | Programming Interface | Revision ID |
        //
        // For, now, we only compare the device's class and the subclass
        class_id  = (uint8_t) ((pci_id_reg >> 24) & 0xff);
        sclass_id = (uint8_t) ((pci_id_reg >> 16) & 0xff);
        if (class_id != pci_class || sclass_id != pci_sclass) {
            ixl_debug("Skipping device of class 0x%x", class_id);
            continue;
        }

        if (i == idx) {
            ixl_debug("Found device of class 0x%x at idx %u", pci_class, idx);
            break;
        }
        else
            i++;
    }

    // Did we fail to reach a device at index dev_idx?
    if (class_id != pci_class) {
        ixl_warn("PCI device at bus index %u is not of expected class %x!",
                 idx, pci_class);
    }

    return(child);
}
