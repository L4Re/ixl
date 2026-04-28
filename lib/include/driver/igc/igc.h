/*****************************************************************************
 *                                                                           *
 *          igc.h - Header for a simple Igc device driver on L4.             *
 *                                                                           *
 * Copyright (C) 2025 Richter Paul <paul.richter@spreewalddreieck.de>        *
 *                                                                           *
 * The igc device driver was developed as part of the bachelor thesis:       *
 * "Generic aspects of porting a Linux Ethernet driver to the                *
 * L4Re microkernel" It is nearly an identical copy of the ixl igb driver    *
 * with changed register definitions. More information about how it works    *
 * can be found there.                                                       *
 *                                                                           *
 * https://github.com/hockeyfriend/                                          *
 * Generic-aspects-of-porting-a-Linux-Ethernet-driver-to-the-L4Re-microkernel*
 *                                                                           *
 *****************************************************************************/


#pragma once

#include <l4/ixl/stats.h>
#include <l4/ixl/memory.h>

#include "igc_type.h"
#include "../../pci.h"

namespace Ixl {

/**
 * Device driver specialized for the igc NIC family (e.g. I225-IT).
 *
 * Note: For now, this software is only a slight adaptation of the E1000 driver,
 * leaving out many features such as RSS (multi-queue receive) etc. Maybe we
 * will add those later, which will make the igc driver much more similar to
 * ixgbe than to the original E1000 one (even though many register definitions
 * stay the same).
 */
class Igc_device : public Ixl_device {
public:
    // set number of queue entries to linux default value
    static const int NUM_RX_QUEUE_ENTRIES = 256;
    static const int NUM_TX_QUEUE_ENTRIES = 256;

    static const int MAX_RX_QUEUE_ENTRIES = 256;
    static const int MAX_TX_QUEUE_ENTRIES = 256;

    static const int PKT_BUF_ENTRY_SIZE  = 2048;

    // Reserve at least twice the RX queue depth of packets for mempools. This
    // is what the driver needs to remain operational as every received packet
    // is immediately replaced with a fresh one in the RX path.
    static const int MIN_MEMPOOL_ENTRIES = 2 * NUM_RX_QUEUE_ENTRIES;

    static const int TX_CLEAN_BATCH      = 32;

    static const uint64_t INTERRUPT_INITIAL_INTERVAL = 1000 * 1000 * 1000;

    std::string get_driver_name(void) {
        return("ixl-igc");
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

    uint32_t get_link_duplex(void);

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

        struct igc_rx_queue *queue =
            ((struct igc_rx_queue *) rx_queues) + qid;

        return queue->mempool->reserve(count);
    }

    // Shrink an RX queue's mempool
    void shrink_rxq_mempool(uint16_t qid, uint32_t count) {
        if (qid >= num_rx_queues)
            return;

        struct igc_rx_queue *queue =
            ((struct igc_rx_queue *) rx_queues) + qid;

        queue->mempool->cancel_reservation(count);
    }

    /*
     * Initializes and returns the Igc device.
     *
     * \param pci_dev PCI device handle received from this task's vbus
     * \param cfg     An Ixl device configuration. See Dev_cfg for details.
     *
     * \return The initialized Igc device.
     */
    static Igc_device* igc_init(L4vbus::Pci_dev &&pci_dev, struct Dev_cfg &cfg);

private:
    // allocated for each rx queue, keeps state for the receive function
    struct igc_rx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile struct igc_rx_desc* descriptors;

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
    struct igc_tx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile struct igc_tx_desc* descriptors;

        // No. of descriptors in the queue
        uint16_t num_entries;
        // position to clean up descriptors that where sent out by the nic
        uint16_t clean_index;
        // position to insert packets for transmission
        uint16_t tx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };

    /***                           Constructor                              ***/
    Igc_device(L4vbus::Pci_dev &&dev, struct Dev_cfg &cfg, uint32_t itr_rate) {
        l4_timeout_s l4tos;     // L4 timeout object with us granularity

        if (cfg.num_rx_queues != 1) {
            ixl_error("Currently, an Igc device supports exactly one receive queue.");
        }
        if (cfg.num_tx_queues != 1) {
            ixl_error("Currently, an Igc device supports exactly one transmit queue.");
        }

        // (see igb for reference)
        num_rx_queues = cfg.num_rx_queues;
        num_tx_queues = cfg.num_tx_queues;

        // Set up IRQ-related data
        if (cfg.irq_timeout_ms < 0) {
            l4tos = l4_timeout_from_us(L4_TIMEOUT_US_NEVER);
        }
        else {
            l4tos = l4_timeout_from_us(cfg.irq_timeout_ms * 1000);
        }

        if (cfg.irq_timeout_ms == 0) {
            interrupts.mode = interrupt_mode::Disable;
        }
        else if (cfg.irq_timeout_ms == -1) {
            interrupts.mode = interrupt_mode::Notify;
        }
        else {
            interrupts.mode = interrupt_mode::Wait;
        }

        interrupts.itr_rate = itr_rate;
        interrupts.timeout  = l4_timeout(l4tos, l4tos);

        // map io memory based on memory address provided by pci register
        pci_dev  = dev;

        // Map BAR0 region
        ixl_debug("Mapping BAR0 I/O memory...");
        baddr[0] = pci_map_bar(pci_dev, 0);

        // Create a DMA space for this device
        create_dma_space();

        if (interrupts.mode != interrupt_mode::Disable) {
            // setup icu capability as before managing interrupts (see igb for
            // reference)
            setup_icu_cap();
            init_msix();
        }

        rx_queues = calloc(num_rx_queues, sizeof(struct igc_rx_queue) + sizeof(void*) * MAX_RX_QUEUE_ENTRIES);
        tx_queues = calloc(num_tx_queues, sizeof(struct igc_tx_queue) + sizeof(void*) * MAX_TX_QUEUE_ENTRIES);
    }

    /***                            Functions                               ***/

    /**
     * Enables an MSI for receive events. We will configure the NIC in a way
     * that an MSI is generated for each packet received, while adhering to the
     * ITR limit that the user can specify upon initializing the driver.
     *
     * \param qid Index of the queue for that the corresponding MSI-X shall be
     *            enabled.
     */
    void enable_rx_interrupt(uint16_t qid);

    /**
     * Disables an MSI for receive events.
     *
     * \param qid Index of the RX queue for that the corresponding MSI-X shall
     *            be disabled.
     */
    void disable_rx_interrupt(uint16_t qid);

    void disable_interrupts(void);

    void init_link(void);

    void start_rx_queue(int queue_id);

    void start_tx_queue(int queue_id);

    void init_rx(void);

    void init_tx(void);

    void wait_for_link(void);

    void reset_and_init(void);

    void init_msix(void);

    /***                          Member variables                          ***/

    // MAC address of this device
    struct mac_address mac_addr;
    // Does mac_addr contain a valid value?
    bool mac_init = false;
};

} // namespace Ixl
