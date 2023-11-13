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

#include "e1000_type.h"
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
    static const int MIN_MEMPOOL_ENTRIES = 512;

    static const int TX_CLEAN_BATCH      = 32;

    // Maximum time span to wait for an EEPROM operation in milliseconds
    static const int EEPROM_MAX_WAIT_MS  = 100;

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
     * @param pci_dev PCI device handle received from this task's vbus
     * @param rx_queues The number of receiver queues.
     * @param tx_queues The number of transmitter queues.
     * @param interrupt_timeout The interrupt timeout in milliseconds
     *      - if set to -1 the interrupt timeout is disabled
     *      - if set to 0 the interrupt is disabled entirely)
     * @return The initialized IXGBE device.
     */
    static E1000_device* e1000_init(L4vbus::Pci_dev&& pci_dev,
                                    uint16_t rx_queues,
                                    uint16_t tx_queues,
                                    int irq_timeout);

    std::string get_driver_name(void) {
        return("ixl-e1000");
    }

private:
    // allocated for each rx queue, keeps state for the receive function
    struct e1000_rx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile struct e1000_rx_desc* descriptors;
        
        // DMA'able memory for storing incoming packets
        struct mempool* mempool;
        
        // No. of descriptors in the queue
        uint16_t num_entries;
        // position we are reading from
        uint16_t rx_index;
        // virtual addresses to map descriptors back to their mbuf for freeing
        void* virtual_addresses[];
    };

    // allocated for each tx queue, keeps state for the transmit function
    struct e1000_tx_queue {
        // DMA'able memory from that the individual descriptors are allocated
        struct dma_memory descr_mem;

        // Array of descriptors backed by descr_mem
        volatile struct e1000_tx_desc* descriptors;
        
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
    E1000_device(L4vbus::Pci_dev&& dev, uint16_t rx_qs, uint16_t tx_qs,
                 bool irq_enabled, uint32_t itr_rate, int irq_timeout_ms) {
        l4_timeout_s l4tos = l4_timeout_from_us(irq_timeout_ms * 1000);

        // Integrity check: The E1000 series does not have multi-queue support
        if (rx_qs != 1)
            ixl_error("An E1000 device supports exactly one receive queue.");
        if (tx_qs != 1)
            ixl_error("An E1000 device supports exactly one transmit queue.");

        num_rx_queues = rx_qs;
        num_tx_queues = tx_qs;

        interrupts.interrupts_enabled = irq_enabled;
        interrupts.itr_rate           = itr_rate;
        interrupts.timeout            = l4_timeout(l4tos, l4tos);
    
        pci_dev  = dev;

        // Do the IRQ-related setup if requested by user
        if (irq_enabled) {
            setup_icu_cap();
            setup_interrupts();
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
    
    /**
     *
     * Read <word_cnt> number of 16-Bit words from the NIC's EEPROM, starting
     * at address <addr>. Note that addressing in the EEPROM is done with an
     * increment of 2 Bytes. <buf> shall point to an array of uint16_t with 
     * a capacity of at least <word_cnt> elements.
     *
     * @param saddr Starting address to read from the NIC's EEPROM.
     * @param word_cnt Number of 2-Byte words to read.
     * @param buf Buffer for storing the returned data.
     *
     * @return 0 on success, 1 otherwise.
     */
    int read_eeprom(uint8_t saddr, uint8_t word_cnt, uint16_t *buf);

    /**
     * Discard all pending interrupts for this device.
     *
     * See also section 13.4.17
     */
    void clear_interrupts(void) {
        get_reg32(addr, E1000_ICR); 
    }

    /**
     * Disabled all interrupts for this device.
     *
     * See also section 13.4.21
     */
    void disable_interrupts(void) {
        ixl_debug("Masking off all IRQs for E1000 device");
        set_reg32(addr, E1000_IMC, 0xffffffff); 
    }

    /**
     * Enables the MSI for receive events. We will configure the NIC in a way
     * that an MSI is generated for each packet received, while adhering to
     * the ITR limit that the user can specify upon initializing the driver.
     */
    void enable_rx_interrupt(void);

    void setup_interrupts(void);

    void init_link(void);

    void start_rx_queue(int queue_id);

    void start_tx_queue(int queue_id);

    void init_rx(void);

    void init_tx(void);

    void wait_for_link(void);

    void reset_and_init(void);

    /***                        Member variables                          ***/
    
    // MAC address of this device
    struct mac_address mac_addr;
    // Does mac_addr contain a valid value?
    bool mac_init = false;
};

}
