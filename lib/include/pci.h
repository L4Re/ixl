#ifndef IXY_PCI_H
#define IXY_PCI_H

#include <stdint.h>

#include <l4/vbus/vbus_pci>

#include <l4/ixylon/device.h>

/****************************************************************************
 *                                                                          *
 *                          function prototypes                             *
 *                                                                          *
 ****************************************************************************/


void enable_dma(L4vbus::Pci_dev& dev);

/**
 * Maps the I/O memory of the first base address register (BAR0) into the
 * address space of this task.
 *
 * @param dev PCI device for that the mapping shall be done.
 *
 * @return The virtual address at which the contents of BAR 0 can be accessed.
 */
uint8_t* pci_map_bar0(L4vbus::Pci_dev& dev);

int pci_open_resource(const char* pci_addr, const char* resource, int flags);

/**
 * Computes the size of the I/O memory that can be accessed by mapping BAR0
 * of the device dev.
 *
 * See also https://wiki.osdev.org/PCI#Base_Address_Registers
 */
l4_uint64_t get_bar0_size(L4vbus::Pci_dev& dev);

/**
 * Gets the physical address of BAR0 of the PCI device dev.
 *
 * See also: https://wiki.osdev.org/PCI#Base_Address_Registers
 */
l4_uint64_t get_bar0_addr(L4vbus::Pci_dev& dev);

/**
 *
 * Tries to acquire a Vbus capability named "vbus" and returns a Pci_dev
 * handle to the idx.th device on this bus that matches its device class to
 * pci_class and its subclass to pci_sclass, respectively. 
 * Note that idx is zero-indexed.
 *
 * TODO: Implement direct addressing by a canonical PCI identifier. This 
 * however would require some engineering effort in IO as well to expose full
 * PCI addresses.
 *
 * @param idx        - Index of the device to search.
 * @param pci_class  - PCI class ID requirement for returned devices.
 * @param pci_sclass - PCI subclass ID requirement for returned devices.
 *
 * @return A handle to the corresponding PCI device.
 */
L4vbus::Pci_dev pci_get_dev(uint32_t idx, uint8_t pci_class, uint8_t pci_sclass);

#endif // IXY_PCI_H
