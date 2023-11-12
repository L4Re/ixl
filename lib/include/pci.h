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


namespace Ixl {

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
 * Returns true if the device has support for MSI-X.
 *
 * See also: https://wiki.osdev.org/PCI#Enabling_MSI-X
 *
 * \param dev The PCI device to check.
 */
bool pcidev_supports_msix(L4vbus::Pci_dev &dev);

/**
 * Returns true if the device has support for MSI.
 *
 * See also: https://wiki.osdev.org/PCI#Enabling_MSI
 *
 * \param dev The PCI device to check.
 */
bool pcidev_supports_msi(L4vbus::Pci_dev &dev);

/**
 * Performs the initial configuration to use MSIs on a PCI device.
 *
 * \param dev The device to configure.
 */
void setup_msi(L4vbus::Pci_dev &dev);

/**
 * Enables the MSI no. irqnum on the PCI level.
 *
 * \param dev    PCI device to be configured.
 * \param irqnum Number of MSI to enable.
 * \param info   Info object about the ICU that this IRQ is controlled by
 *               (contains addressing info etc.)
 */
void pcidev_enable_msi(L4vbus::Pci_dev &dev, uint32_t irqnum,
                       l4_icu_msi_info_t info);

/**
 * Gets information necessary for accessing the MSI-X table of a PCI device.
 *
 * \param dev             The device that shall be examined.
 * \param bir[out]        Base address register the MSI-X table lives in.
 * \param table_offs[out] Offset needed for locating the MSI-X table in the BAR.
 * \param table_size[out] Number of entries in the table (N - 1 encoded).
 */
void pcidev_get_msix_info(L4vbus::Pci_dev &dev, uint32_t *bir,
                          uint32_t *table_offs, uint32_t *table_size);

/**
 * Enables the MSI-X no. irqnum on the PCI level. The last three parameters are
 * usually obtained by a call to pcidev_get_msix_info.
 *
 * \param irqnum     Number of MSI-X to enable.
 * \param info       Info object about the ICU that this IRQ is controlled by
 *                   (contains addressing info etc.)
 * \param bar_addr   VMA of the BAR that the table lives in (needs to be
 *                   mapped to this AS beforehand).
 * \param table_offs Offset needed for locating the MSI-X table in the BAR.
 * \param table_size Number of entries in the table (N - 1 encoded).
 */
void pcidev_enable_msix(uint32_t irqnum, l4_icu_msi_info_t info,
                        uint8_t *bar_addr, uint32_t table_offs,
                        uint32_t table_size);

/**
 * Performs the initial configuration to use MSI-Xs on a PCI device.
 *
 * \param dev The device to configure.
 */
void setup_msix(L4vbus::Pci_dev &dev);

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
 * @param vbus       Vbus to search for devices.
 * @param idx        Index of the device to search.
 * @param pci_class  PCI class ID requirement for returned devices.
 * @param pci_sclass PCI subclass ID requirement for returned devices.
 *
 * @return A handle to the corresponding PCI device.
 */
L4vbus::Pci_dev pci_get_dev(L4::Cap<L4vbus::Vbus> vbus,
                            uint32_t idx, uint8_t pci_class,
                            uint8_t pci_sclass);

} // namespace Ixl

#endif // IXY_PCI_H
