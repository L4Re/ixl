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

#include <l4/ixylon/log.h>

#include "pci.h"

void remove_driver(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/driver/unbind", pci_addr);
	int fd = open(path, O_WRONLY);
	if (fd == -1) {
		ixl_debug("no driver loaded");
		return;
	}
	if (write(fd, pci_addr, strlen(pci_addr)) != (ssize_t) strlen(pci_addr)) {
		ixl_warn("failed to unload driver for device %s", pci_addr);
	}
	check_err(close(fd), "close");
}

void enable_dma(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/config", pci_addr);
	int fd = check_err(open(path, O_RDWR), "open pci config");
	// write to the command register (offset 4) in the PCIe config space
	// bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
	assert(lseek(fd, 4, SEEK_SET) == 4);
	uint16_t dma = 0;
	assert(read(fd, &dma, 2) == 2);
	dma |= 1 << 2;
	assert(lseek(fd, 4, SEEK_SET) == 4);
	assert(write(fd, &dma, 2) == 2);
	check_err(close(fd), "close");
}

uint8_t* pci_map_resource(const char* pci_addr) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/resource0", pci_addr);
	ixl_debug("Mapping PCI resource at %s", path);
	remove_driver(pci_addr);
	enable_dma(pci_addr);
	int fd = check_err(open(path, O_RDWR), "open pci resource");
	struct stat stat;
	check_err(fstat(fd, &stat), "stat pci resource");
	uint8_t* hw = (uint8_t*) check_err(mmap(NULL, stat.st_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0), "mmap pci resource");
	check_err(close(fd), "close pci resource");
	return hw;
}

int pci_open_resource(const char* pci_addr, const char* resource, int flags) {
	char path[PATH_MAX];
	snprintf(path, PATH_MAX, "/sys/bus/pci/devices/%s/%s", pci_addr, resource);
	ixl_debug("Opening PCI resource at %s", path);
	int fd = check_err(open(path, flags), "open pci resource");
	return fd;
}

/* Acquires a PCI devices at a certain index on the "vbus" capability.      */
L4vbus::Pci_dev pci_get_dev(uint32_t idx, uint8_t pci_class, uint8_t pci_sclass)
{
    auto vbus = L4Re::chkcap(L4Re::Env::env()->get_cap<L4vbus::Vbus>("vbus"),
                             "Get vbus capability.", -L4_ENOENT);

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
