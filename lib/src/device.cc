#include <sys/file.h>

#include <l4/vbus/vbus_pci>

#include <l4/ixylon/device.h>
#include "driver/ixgbe/ixgbe.h"
// #include "driver/virtio.h"
#include "pci.h"

using namespace Ixl;

Ixl_device* Ixl_device::ixl_init(const char* pci_addr, uint16_t rx_queues,
                                 uint16_t tx_queues, int irq_timeout) {
    check_err(pci_addr != NULL, "pci_addr must not be NULL!");
    
    // Only take the function identifier (last digit) of the canonical PCI 
    // address as an index for device discovery
    uint32_t dev_idx = atoi(pci_addr + strlen(pci_addr) - 1);

    // Read PCI configuration space
    // For VFIO, we could access the config space another way
    // (VFIO_PCI_CONFIG_REGION_INDEX). This is not needed, though, because
    // every config file should be world-readable, and here we
    // only read the vendor and device id.
    uint32_t vendor_id;
    uint32_t device_id;
    
    // We search for Ethernet devices only (class 0x2, subclass 0x0)
    L4vbus::Pci_dev dev = pci_get_dev(dev_idx, 0x02, 0x00);
    check_err(dev.cfg_read(0, &vendor_id, 16), "Failed to read PCI vendor ID");
    check_err(dev.cfg_read(2, &device_id, 16), "Failed to read PCI device ID");

    ixl_debug("Vendor ID is %x, Device ID is %x", vendor_id, device_id);
    if (vendor_id == 0x1af4 && device_id >= 0x1000) {
        ixl_error("Virtio driver is currently broken. Aborting...");
        // return virtio_init(pci_addr, rx_queues, tx_queues);
    } else {
        // Our best guess is to try ixgbe
        ixl_info("Trying ixgbe...");
        return Ixgbe_device::ixgbe_init(pci_addr, std::move(dev), rx_queues,
                                        tx_queues, irq_timeout);
    }
}
