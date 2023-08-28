/*****************************************************************************
 *                                                                           *
 *     E1000_device - Header for a simple e1000 NIC device driver on L4.     *
 *                                                                           *
 * Copyright (C) 2023 Till Miemietz <till.miemietz@barkhauseninstitut.org>   *
 *                                                                           *
 *****************************************************************************/

#pragma once

#include <stdbool.h>
#include <l4/ixylon/stats.h>

#include <l4/ixylon/memory.h>

#include "../../pci.h"

namespace Ixl {

/**
 * Device driver specialized for the e1000 NIC family.
 *
 * Note: For now, this driver only supports polling. IRQ support is to be
 *       added later.
 */
class E1000_device : public Ixl_device {
public:
    // Taken from the linux kernel, file:
    // /drivers/net/ethernet/intel/e1000/e1000.h
    // Note that there is one series of e1000, that supports 4096 entries per
    // queue, but we don't support it for now.
    static const int MAX_RX_QUEUE_ENTRIES = 256;
    static const int MAX_TX_QUEUE_ENTRIES = 256;

    // The sum of these constants defines the size of the overall memory pool
    // used for receiving / sending packets. For now, we set it to the 
    // maximum values supported.
    static const int NUM_RX_QUEUE_ENTRIES = 256;
    static const int NUM_TX_QUEUE_ENTRIES = 256;

    static const int PKT_BUF_ENTRY_SIZE  = 2048;
    static const int MIN_MEMPOOL_ENTRIES = 4096;

    static const int TX_CLEAN_BATCH = 32;

    // TODO: Unused for now, reactivate later on.
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
     * Initializes and returns the E1000 device.
     * @param pci_addr The PCI address of the device.
     * @param pci_dev PCI device handle received from this task's vbus
     * @param rx_queues The number of receiver queues.
     * @param tx_queues The number of transmitter queues.
     * @param interrupt_timeout The interrupt timeout in milliseconds
     *      - if set to -1 the interrupt timeout is disabled
     *      - if set to 0 the interrupt is disabled entirely)
     * @return The initialized IXGBE device.
     */
    static E1000_device* e1000_init(const char *pci_addr,
                                    L4vbus::Pci_dev&& pci_dev,
                                    uint16_t rx_queues,
                                    uint16_t tx_queues,
                                    int irq_timeout);

    std::string get_driver_name(void) {
        return("ixl-e1000");
    }

private:
    // allocated for each rx queue, keeps state for the receive function
    struct e1000_rx_queue {
        volatile union ixgbe_adv_rx_desc* descriptors;
        struct mempool* mempool;
        uint16_t num_entries;
        // position we are reading from
        uint16_t rx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };

    // allocated for each tx queue, keeps state for the transmit function
    struct e1000_tx_queue {
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
    E1000_device(const char* pci_address, L4vbus::Pci_dev&& dev,
                 uint16_t rx_qs, uint16_t tx_qs, bool irq_enabled,
                 uint32_t itr_rate, int irq_timeout_ms) {

        // Integrity check: The E1000 series does not have multi-queue support
        if (rx_qs != 1)
            ixl_error("An E1000 device supports exactly one receive queue.");
        if (tx_qs != 1)
            ixl_error("An E1000 device supports exactly one transmit queue.");

        num_rx_queues = rx_qs;
        num_tx_queues = tx_qs;

        interrupts.interrupts_enabled = irq_enabled;
        interrupts.itr_rate           = itr_rate;
        interrupts.timeout_ms         = irq_timeout_ms;
    
        pci_addr = strdup(pci_address);
        pci_dev  = dev;

        // Temporary hack to indicate absence of IRQ implementation
        if (irq_enabled) {
            ixl_error("IRQ feature currently not implemented.");
            // setup_interrupts();
        }

        // Map BAR0 region
        ixl_debug("Mapping BAR0 I/O memory...");
        addr = pci_map_bar0(pci_dev);

        // Create a DMA space for this device
        create_dma_space();

        rx_queues = calloc(rx_qs, sizeof(struct e1000_rx_queue) + sizeof(void*) * MAX_RX_QUEUE_ENTRIES);
        tx_queues = calloc(tx_qs, sizeof(struct e1000_tx_queue) + sizeof(void*) * MAX_TX_QUEUE_ENTRIES);
    }

    /***                           Functions                              ***/

    void setup_interrupts();

    void init_link(void);

    void start_rx_queue(int queue_id);

    void start_tx_queue(int queue_id);

    void init_rx(void);

    void init_tx(void);

    void wait_for_link(void);

    void reset_and_init(void);

    /***                        Member variables                          ***/
    
    // Memory address at which the I/O memory described in BAR0 got mapped.
    uint8_t* addr;
    
    void*    rx_queues;
    void*    tx_queues;
};

}
