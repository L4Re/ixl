#ifndef IXY_IXGBE_H
#define IXY_IXGBE_H

#include <stdbool.h>
#include <l4/ixylon/stats.h>

#include <l4/ixylon/memory.h>

#include "ixgbe_type.h"
#include "../pci.h"

namespace Ixl {

/**
 * Device driver specialized for the ixgbe NIC family.
 */
class Ixgbe_device : public Ixl_device {
public:
    static const int MAX_RX_QUEUE_ENTRIES = 4096;
    static const int MAX_TX_QUEUE_ENTRIES = 4096;

    static const int NUM_RX_QUEUE_ENTRIES = 512;
    static const int NUM_TX_QUEUE_ENTRIES = 512;

    static const int PKT_BUF_ENTRY_SIZE = 2048;
    static const int MIN_MEMPOOL_ENTRIES = 4096;

    static const int TX_CLEAN_BATCH = 32;

    static const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

    uint32_t rx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                      uint32_t num_bufs);

    uint32_t tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                      uint32_t num_bufs);

    void read_stats(struct device_stats *stats);

    void set_promisc(bool enabled);

    uint32_t get_link_speed(void);

    struct mac_address get_mac_addr(void);

    void set_mac_addr(struct mac_address mac);

    /**
     * Initializes and returns the IXGBE device.
     * @param pci_addr The PCI address of the device.
     * @param rx_queues The number of receiver queues.
     * @param tx_queues The number of transmitter queues.
     * @param interrupt_timeout The interrupt timeout in milliseconds
     *      - if set to -1 the interrupt timeout is disabled
     *      - if set to 0 the interrupt is disabled entirely)
     * @return The initialized IXGBE device.
     */
    static Ixgbe_device* ixgbe_init(const char *pci_addr,
                                    uint16_t rx_queues,
                                    uint16_t tx_queues,
                                    int irq_timeout);

    std::string get_driver_name(void) {
        return("ixl-ixgbe");
    }

private:
    // allocated for each rx queue, keeps state for the receive function
    struct ixgbe_rx_queue {
        volatile union ixgbe_adv_rx_desc* descriptors;
        struct mempool* mempool;
        uint16_t num_entries;
        // position we are reading from
        uint16_t rx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };

    // allocated for each tx queue, keeps state for the transmit function
    struct ixgbe_tx_queue {
        volatile union ixgbe_adv_tx_desc* descriptors;
        uint16_t num_entries;
        // position to clean up descriptors that where sent out by the nic
        uint16_t clean_index;
        // position to insert packets for transmission
        uint16_t tx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };


    /***                           Constructor                            ***/
    Ixgbe_device(const char* pci_address,
                 uint16_t rx_qs, uint16_t tx_qs, bool irq_enabled,
                 uint32_t itr_rate, int irq_timeout_ms) {
        num_rx_queues = rx_qs;
        num_tx_queues = tx_qs;

        interrupts.interrupts_enabled = irq_enabled;
        interrupts.itr_rate           = itr_rate;
        interrupts.timeout_ms         = irq_timeout_ms;
    
        pci_addr = strdup(pci_address);

        // TODO: Purge the vfio stuff.
        if (!vfio && irq_timeout_ms != 0) {
            ixl_warn("Interrupts requested but VFIO not available: "
                     "Disabling Interrupts!");
            interrupts.interrupts_enabled = false;  
        }

        // TODO: Purge the vfio stuff
        // Map BAR0 region
        if (vfio) {
            ixl_error("VFIO not supported on L4");
            /*
            debug("mapping BAR0 region via VFIO...");
            dev->addr = vfio_map_region(dev->ixy.vfio_fd, VFIO_PCI_BAR0_REGION_INDEX);
            // initialize interrupts for this device
            setup_interrupts(dev);
            */
        } else {
            ixl_debug("mapping BAR0 region via pci file...");
            addr = pci_map_resource(pci_addr);
        }

        rx_queues = calloc(rx_qs, sizeof(struct ixgbe_rx_queue) + sizeof(void*) * MAX_RX_QUEUE_ENTRIES);
        tx_queues = calloc(tx_qs, sizeof(struct ixgbe_tx_queue) + sizeof(void*) * MAX_TX_QUEUE_ENTRIES);
    }

    /***                           Functions                              ***/

    /**
     * Set the IVAR registers, mapping interrupt causes to vectors
     * @param dev pointer to device
     * @param direction 0 for Rx, 1 for Tx
     * @param queue queue to map the corresponding interrupt to
     * @param msix_vector the vector to map to the corresponding queue
     */
    void set_ivar(int8_t direction, int8_t queue, int8_t msix_vector) {
        u32 ivar, index;
        msix_vector |= IXGBE_IVAR_ALLOC_VAL;
        index = ((16 * (queue & 1)) + (8 * direction));
        ivar = get_reg32(addr, IXGBE_IVAR(queue >> 1));
        ivar &= ~(0xFF << index);
        ivar |= (msix_vector << index);
        set_reg32(addr, IXGBE_IVAR(queue >> 1), ivar);
    }

    /**
     * Clear all interrupt masks for all queues.
     * @param dev The device.
     */
    void clear_interrupts(void) {
        // Clear interrupt mask
        set_reg32(addr, IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
        get_reg32(addr, IXGBE_EICR);
    }

    /**
     * Clear interrupt for queue.
     * @param dev The device.
     * @param queue_id The ID of the queue to clear.
     */
    void clear_interrupt(uint16_t queue_id) {
        // Clear interrupt mask
        set_reg32(addr, IXGBE_EIMC, 1 << queue_id);
        get_reg32(addr, IXGBE_EICR);
    }

    /**
     * Disable all interrupts for all queues.
     * @param dev The device.
     */
    void disable_interrupts(void) {
        // Clear interrupt mask to stop from interrupts being generated
        set_reg32(addr, IXGBE_EIMS, 0x00000000);
        clear_interrupts();
    }

    /**
     * Disable interrupt for queue
     * @param dev
     * @param queue_id The ID of the queue to disable.
     */
    void disable_interrupt(uint16_t queue_id) {
        // Clear interrupt mask to stop from interrupts being generated
        u32 mask = get_reg32(addr, IXGBE_EIMS);
        mask &= ~(1 << queue_id);
        set_reg32(addr, IXGBE_EIMS, mask);
        clear_interrupt(queue_id);
        ixl_debug("Using polling");
    }

    /**
     * Enable MSI interrupt for queue.
     * @param dev The device.
     * @param queue_id The ID of the queue to enable.
     */
    void enable_msi_interrupt(uint16_t queue_id);

    /**
     * Enable MSI-X interrupt for queue.
     * @param dev The device.
     * @param queue_id The ID of the queue to enable.
     */
    void enable_msix_interrupt(uint16_t queue_id);

    /**
     * Enable MSI or MSI-X interrupt for queue depending on which is
     * supported (Prefer MSI-x).
     * @param dev The device.
     * @param queue_id The ID of the queue to enable.
     */
    void enable_interrupt(uint16_t queue_id);

    /**
     * Setup interrupts by enabling VFIO interrupts.
     * @param dev The device.
     */
    void setup_interrupts();

    // see section 4.6.4
    void init_link(void);

    void start_rx_queue(int queue_id);

    void start_tx_queue(int queue_id);

    // see section 4.6.7
    // it looks quite complicated in the data sheet, but it's actually really
    // easy because we don't need fancy features
    void init_rx(void);

    // see section 4.6.8
    void init_tx(void);

    void wait_for_link(void);

    // see section 4.6.3
    void reset_and_init(void);

    /***                        Member variables                          ***/
    uint8_t* addr;
    void*    rx_queues;
    void*    tx_queues;
};

}

#endif //IXY_IXGBE_H
