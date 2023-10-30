#include <assert.h>
#include <errno.h>
#include <linux/limits.h>
#include <stdio.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <l4/re/env>
#include <l4/re/error_helper>
#include <l4/vbus/vbus>
#include <l4/vbus/vbus_pci>
#include <l4/vbus/vbus_interfaces.h>

#include <l4/ixylon/log.h>

#include "pci.h"

/* Compute the size of the I/O memory accessible through BAR0               */
l4_uint64_t get_bar0_size(L4vbus::Pci_dev& dev) {
    // TODO: According to OSdev wiki, one should also disable the I/O and
    //       memory decode bytes in the command register before executing
    //       this operation...
    l4_uint64_t bar_size = 0;

    l4_uint32_t lsb = ~((l4_uint32_t) 0x0);
    l4_uint32_t msb = ~((l4_uint32_t) 0x0);
    l4_uint32_t lsb_old;
    l4_uint32_t msb_old;

    // Save original content of BAR0
    L4Re::chksys(dev.cfg_read(0x10, &lsb_old, 32));
    L4Re::chksys(dev.cfg_read(0x14, &msb_old, 32));
    
    // Check if the device is memory but 32-bit addressed (Bit 2 != 1))
    if (! (lsb_old & 0x00000004)) {
        // Write all-ones to the register and read back the BAR value
        L4Re::chksys(dev.cfg_write(0x10, lsb, 32));
        L4Re::chksys(dev.cfg_read(0x10, &lsb, 32));

        // restore the original register contents
        L4Re::chksys(dev.cfg_write(0x10, lsb_old, 32));

        // Now compute the size of the BAR memory. Remember to clear the lower
        // four bits of the LSB register part
        lsb      = lsb & 0xfffffff0;
        bar_size = (l4_uint64_t) ~lsb;
    }
    else {
        // Device is 64-bit addressed, so the MSB part of the BAR size is
        // contained in the next MSB.

        // Write all-ones to the register and read back the BAR value
        L4Re::chksys(dev.cfg_write(0x10, lsb, 32));
        L4Re::chksys(dev.cfg_write(0x14, msb, 32));
        L4Re::chksys(dev.cfg_read(0x10, &lsb, 32));
        L4Re::chksys(dev.cfg_read(0x14, &msb, 32));

        // restore the original register contents
        L4Re::chksys(dev.cfg_write(0x10, lsb_old, 32));
        L4Re::chksys(dev.cfg_write(0x14, msb_old, 32));

        // Now compute the size of the BAR memory. Remember to clear the lower
        // four bits of the LSB register part
        lsb      = lsb & 0xfffffff0;
        bar_size = ((l4_uint64_t) msb << 32) | lsb;
        bar_size = ~bar_size;
    }

    return bar_size + 1;
}

/* Get the physical address of BAR0                                         */
l4_uint64_t get_bar0_addr(L4vbus::Pci_dev& dev) {
    l4_uint32_t lsb;
    l4_uint32_t msb;

    L4Re::chksys(dev.cfg_read(0x10, &lsb, 32));
    
    // Always mask the lower bits of the LSB (mapping is page-aligned)

    // I/O Spaces are always 32-bit addressed
    if (lsb & 0x00000001)
        return ((l4_uint64_t) lsb) & 0xfffffffffffff000UL;

    // Check if the device is memory but 32-bit addressed (Bit 2 != 1))
    if (! (lsb & 0x00000004))
        return ((l4_uint64_t) lsb) & 0xfffffffffffff000UL;

    // Device is 64-bit addressed, read the next BAR to obtain the MSB part of
    // the address
    L4Re::chksys(dev.cfg_read(0x14, &msb, 32));

    return (((l4_uint64_t) msb) << 32 | lsb) & 0xfffffffffffff000UL;
}

void enable_dma(L4vbus::Pci_dev& dev) {
    l4_uint32_t cmd_reg = 0;                  // Value of PCI command register
    
    // write to the command register (offset 4) in the PCIe config space
    // bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
    L4Re::chksys(dev.cfg_read(0x4, &cmd_reg, 16));
    cmd_reg |= 1 << 2;
    L4Re::chksys(dev.cfg_write(0x4, cmd_reg, 16));
}

uint8_t* pci_map_bar0(L4vbus::Pci_dev& dev) {
    l4_addr_t   iomem_addr;       // Address for mapping the I/O memory of BAR0
    l4_uint64_t iomem_size;       // Size of memory accessible through BAR0
    l4_uint64_t bar_addr;         // Physical address contained in BAR0

    enable_dma(dev);
    
    // Obtain information about the device (e.g. no. of resources & their types)
    l4vbus_device_t devinfo;
    L4Re::chksys(dev.device(&devinfo));

    ixl_debug("dev has %u resources", devinfo.num_resources);
    
    // Iterate through list of device resoures. For now, we only care about
    // the I/O memory that can be mapped through BAR0
    // TODO: Can be we sure that L4 returns BAR0 as the first I/O memory
    //       resource if multiple I/O memory windows are available?
    unsigned int i;
    l4vbus_resource_t res;
    for (i = 0; i < devinfo.num_resources; i++) {
        L4Re::chksys(dev.get_resource(i, &res));

        if (res.type == L4VBUS_RESOURCE_MEM)
            break;

        ixl_debug("Skipping resource of type %u...", res.type);
    }

    iomem_size = get_bar0_size(dev);
    bar_addr   = get_bar0_addr(dev);

    ixl_debug("bar0 addr = %llx, bar0 size = %llx", bar_addr, iomem_size);

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

    ixl_debug("Mapped bar0 to address %lx", iomem_addr);
    return (uint8_t *) iomem_addr;
}

int pci_open_resource(const char* pci_addr, const char* resource, int flags) {
    char path[PATH_MAX];
    snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/%s", pci_addr, resource);
    ixl_debug("Opening PCI resource at %s", path);
    int fd = check_err(open(path, flags), "open pci resource");
    return fd;
}

/* Acquires a PCI devices at a certain index on the "vbus" capability.      */
L4vbus::Pci_dev pci_get_dev(L4::Cap<L4vbus::Vbus> vbus,
                            uint32_t idx, uint8_t pci_class, uint8_t pci_sclass)
{
    ixl_debug("Starting device discovery...");
    auto root = vbus->root();
    L4vbus::Pci_dev child;

    unsigned int i         = 0;
    uint8_t      class_id  = 0;
    uint8_t      sclass_id = 0;
    // Iterate the vbus until we found the device at index dev_idx
    while (root.next_device(&child) == L4_EOK) {
        uint32_t pci_id_reg = 0;

        // Make sure that the device found matches the desired dev class
        check_err(child.cfg_read(8, &pci_id_reg, 32), 
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
    if (class_id != pci_class)
        ixl_error("Failed to open PCI device at bus index %u", idx);

    return(child);
}
