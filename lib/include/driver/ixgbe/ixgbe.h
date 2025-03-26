#ifndef IXY_IXGBE_H
#define IXY_IXGBE_H

#include <stdbool.h>
#include <l4/ixl/stats.h>

#include <l4/ixl/memory.h>

#include "ixgbe_type.h"
#include "../../pci.h"

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

    // 2048 as pktbuf size is strictly speaking incorrect:
    // we need a few headers (1 cacheline), so there's only 1984 bytes left
    // for the device but the 82599 can only handle sizes in increments of 1 kb.
    // This is fine since our max packet size is the default MTU of 1518.
    // However, this has to be fixed if jumbo frames are to be supported.
    static const int PKT_BUF_ENTRY_SIZE = 2048;

    // TODO: Check this limit
    static const int MIN_MEMPOOL_ENTRIES = 4096;

    static const int TX_CLEAN_BATCH = 32;

    static const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

    std::string get_driver_name(void) {
        return("ixl-ixgbe");
    }

    inline uint32_t get_max_frame_size(void) {
        // Max. frame size of Ethernet is 1518 and we offload the CRC generation
        // so software can write four Bytes less...
        return 1514;
    }

    uint32_t rx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                      uint32_t num_bufs);

    uint32_t tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                      uint32_t num_bufs);

    void read_stats(struct device_stats *stats);

    void set_promisc(bool enabled);

    uint32_t get_link_speed(void);

    struct mac_address get_mac_addr(void);

    void set_mac_addr(struct mac_address mac);

    bool check_recv_irq(uint16_t qid) override;

    void ack_recv_irq(uint16_t qid) override;

    // Get the number of descriptors per RX queue
    uint32_t get_rx_queue_depth(void) {
        return NUM_RX_QUEUE_ENTRIES;
    }

    // Get the number of descriptors per TX queue
    uint32_t get_tx_queue_depth(void) {
        return NUM_TX_QUEUE_ENTRIES;
    }

    // Extend an RX queue's mempool
    bool extend_rxq_mempool(uint16_t qid, uint32_t count) {
        if (qid >= num_rx_queues)
            return false;

        struct ixgbe_rx_queue *queue =
            ((struct ixgbe_rx_queue *) rx_queues) + qid;

        return queue->mempool->reserve(count);
    }

    // Shrink an RX queue's mempool
    void shrink_rxq_mempool(uint16_t qid, uint32_t count) {
        if (qid >= num_rx_queues)
            return;

        struct ixgbe_rx_queue *queue =
            ((struct ixgbe_rx_queue *) rx_queues) + qid;

        queue->mempool->cancel_reservation(count);
    }

    /**
     * Initializes and returns the IXGBE device.
     *
     * \param pci_dev PCI device handle received from this task's vbus.
     * \param cfg     An Ixl device configuration. See Dev_cfg for details.
     *
     * \return The initialized IXGBE device.
     */
    static Ixgbe_device* ixgbe_init(L4vbus::Pci_dev&& pci_dev,
                                    struct Dev_cfg &cfg);

private:
    // allocated for each rx queue, keeps state for the receive function
    struct ixgbe_rx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile union ixgbe_adv_rx_desc* descriptors;

        // DMA'able memory for storing incoming packets
        Mempool* mempool;

        // No. of descriptors in the queue
        uint16_t num_entries;
        // position we are reading from
        uint16_t rx_index;
        // True if this RX queue contains descriptors not yet processed by the
        // driver. We use this flag to skip IRQ receive operations if necessary.
        bool rx_pending = false;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };

    // allocated for each tx queue, keeps state for the transmit function
    struct ixgbe_tx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile union ixgbe_adv_tx_desc* descriptors;

        // No. of descriptors in the queue
        uint16_t num_entries;
        // position to clean up descriptors that where sent out by the nic
        uint16_t clean_index;
        // position to insert packets for transmission
        uint16_t tx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };


    /***                           Constructor                            ***/
    Ixgbe_device(L4vbus::Pci_dev&& dev, struct Dev_cfg &cfg, uint32_t itr_rate){
        l4_timeout_s l4tos;     // L4 timeout object with us granularity

        num_rx_queues = cfg.num_rx_queues;
        num_tx_queues = cfg.num_tx_queues;

        // Set up IRQ-related data
        if (cfg.irq_timeout_ms < 0)
            l4tos = l4_timeout_from_us(L4_TIMEOUT_US_NEVER);
        else
            l4tos = l4_timeout_from_us(cfg.irq_timeout_ms * 1000);

        if (cfg.irq_timeout_ms == 0)
            interrupts.mode = interrupt_mode::Disable;
        else if (cfg.irq_timeout_ms == -1)
            interrupts.mode = interrupt_mode::Notify;
        else
            interrupts.mode = interrupt_mode::Wait;

        interrupts.itr_rate = itr_rate;
        interrupts.timeout  = l4_timeout(l4tos, l4tos);

        pci_dev = dev;

        // Map BAR0 region
        ixl_debug("Mapping BAR0 I/O memory...");
        baddr[0] = pci_map_bar(pci_dev, 0);

        // Create a DMA space for this device
        create_dma_space();

        // Temporary hack to indicate absence of IRQ implementation
        if (cfg.irq_timeout_ms != 0) {
            setup_icu_cap();
            setup_interrupts();
        }

        rx_queues = calloc(num_rx_queues, sizeof(struct ixgbe_rx_queue) + sizeof(void*) * MAX_RX_QUEUE_ENTRIES);
        tx_queues = calloc(num_tx_queues, sizeof(struct ixgbe_tx_queue) + sizeof(void*) * MAX_TX_QUEUE_ENTRIES);
    }

    /***                           Functions                              ***/

    /**
     * Set the IVAR registers, mapping interrupt causes to vectors
     * @param direction 0 for Rx, 1 for Tx
     * @param queue queue to map the corresponding interrupt to
     * @param msix_vector the vector to map to the corresponding queue
     */
    void set_ivar(int8_t direction, int8_t queue, int8_t msix_vector) {
        u32 ivar, index;
        msix_vector |= IXGBE_IVAR_ALLOC_VAL;
        index = ((16 * (queue & 1)) + (8 * direction));
        ivar = get_reg32(baddr[0], IXGBE_IVAR(queue >> 1));
        ivar &= ~(0xFF << index);
        ivar |= (msix_vector << index);
        set_reg32(baddr[0], IXGBE_IVAR(queue >> 1), ivar);
    }

    /**
     * Clear all interrupt masks for all queues.
     */
    void clear_interrupts(void) {
        // Clear interrupt mask
        set_reg32(baddr[0], IXGBE_EIMC, IXGBE_IRQ_CLEAR_MASK);
        get_reg32(baddr[0], IXGBE_EICR);
    }

    /**
     * Clear interrupt for queue.
     * @param queue_id The ID of the queue to clear.
     */
    void clear_interrupt(uint16_t queue_id) {
        // Clear interrupt mask
        set_reg32(baddr[0], IXGBE_EIMC, 1 << queue_id);
        get_reg32(baddr[0], IXGBE_EICR);
    }

    /**
     * Disable all interrupts for all queues.
     */
    void disable_interrupts(void) {
        // Clear interrupt mask to stop from interrupts being generated
        set_reg32(baddr[0], IXGBE_EIMS, 0x00000000);
        clear_interrupts();
    }

    /**
     * Disable interrupt for queue
     * @param queue_id The ID of the queue to disable.
     */
    void disable_interrupt(uint16_t queue_id) {
        // Clear interrupt mask to stop from interrupts being generated
        u32 mask = get_reg32(baddr[0], IXGBE_EIMS);
        mask &= ~(1 << queue_id);
        set_reg32(baddr[0], IXGBE_EIMS, mask);
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
     * Setup interrupts by enabling MSI / MSI-X at the PCI card.
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
};

}

#endif //IXY_IXGBE_H
