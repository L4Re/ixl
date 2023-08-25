#ifndef IXY_PCI_H
#define IXY_PCI_H

#include <stdint.h>

#include <l4/vbus/vbus_pci>

struct ixy_device;

/****************************************************************************
 *                                                                          *
 *                          function prototypes                             *
 *                                                                          *
 ****************************************************************************/


void remove_driver(const char* pci_addr);
void enable_dma(const char* pci_addr);
uint8_t* pci_map_resource(const char* bus_id);
int pci_open_resource(const char* pci_addr, const char* resource, int flags);

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
