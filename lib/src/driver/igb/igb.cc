/***************************************************************************** 
 *                                                                           * 
 * Igb_device - Implementation of a simple Igb NIC device driver on L4.      * 
 *                                                                           * 
 * Copyright (C) 2023 Till Miemietz <till.miemietz@barkhauseninstitut.org>   * 
 *                                                                           * 
 *****************************************************************************/


#include <l4/re/env>
#include <l4/re/error_helper>

#include "driver/igb/igb.h"

using namespace Ixl;

/****************************************************************************
 *                                                                          *
 *                        function implementation                           *
 *                                                                          *
 ****************************************************************************/


/* Enables a receive interrupt of the NIC.                                  */
void Igb_device::enable_rx_interrupt(uint16_t qid) {
    uint32_t msi_vec = interrupts.queues[qid].msi_vec;

    // Limit the ITR to prevent IRQ storms
    set_reg32(addr, IGB_EITR + 4 * msi_vec,
              (interrupts.itr_rate & 0x00001fff) << 2);

    // Unmask and enable the receive timer interrupt cause
    clear_flags32(addr, IGB_EIMC, 1 << msi_vec);
    set_flags32(addr, IGB_EIMS, 1 << msi_vec);
}

/* Disables a receive interrupt of the NIC.                                 */
void Igb_device::disable_rx_interrupt(uint16_t qid) {
    uint32_t msi_vec = interrupts.queues[qid].msi_vec;

    // Unmask and enable the receive timer interrupt cause
    set_flags32(addr, IGB_EIMC, 1 << msi_vec);
    clear_flags32(addr, IGB_EIMS, 1 << msi_vec);
}

/* Read data from the NIC's EEPROM                                          */
int Igb_device::read_eeprom(uint8_t saddr, uint8_t word_cnt, uint16_t *buf) {
    for (int i = 0; i < word_cnt; i++) {
        int      j;
        uint8_t  cur_addr  = saddr + i;

        uint32_t reg_value = 0 | (cur_addr << IGB_EERD_ADDR_SHIFT) |
                                 IGB_EERD_START;
        
        // Trigger the EEPROM read by writing the EERD register, then wait
        // for the result to show up
        set_reg32(addr, IGB_EERD, reg_value);
        
        for (j = 0; j < EEPROM_MAX_WAIT_MS; j++) {
            usleep(1000);
            reg_value = get_reg32(addr, IGB_EERD);

            // Success is indicated via a bit flag
            if (reg_value & IGB_EERD_DONE)
                break;
        }

        // Check whether we reached the timeout
        if (j == EEPROM_MAX_WAIT_MS)    
            return -1;
    
        // Save the data from the EERD register
        buf[i] = (uint16_t) (reg_value >> IGB_EERD_DATA_SHIFT);
    }

    return 0;
}

void Igb_device::setup_interrupts(void) {
    if (! interrupts.interrupts_enabled) {
        return;
    }

    interrupts.queues = (struct interrupt_queue*) malloc(num_rx_queues * sizeof(struct interrupt_queue));

    // FIXME: We should rely on MSI-X / MSI only for Igb devices, also these
    //        NIC have more than one interrupt available
    //
    // Determine type of interrupt available at the device. We will only go
    // with MSI-X in non-SR-IOV mode.
    if (pcidev_supports_msix(pci_dev)) {
        uint32_t bir;               // BAR location of MSI-X table
        uint32_t table_offs;        // Offset of MSI-X table in BAR
        uint32_t table_size;        // Size of MSI-X table
        uint32_t ivar;              // Content of IVAR register

        ixl_info("Using MSI-X interrupts...");
        interrupts.interrupt_type = IXL_IRQ_MSIX;
        setup_msix(pci_dev);

        pcidev_get_msix_info(pci_dev, &bir, &table_offs, &table_size);
        // FIXME: Right now we rely on the MSI-X table being located in BAR 0
        if (bir != 0) {
            ixl_error("Can not access MSI-X table in BAR %u.", bir);
        }

        // Configure the NIC to use multiple MSI-X mode (one IRQ per queue)
        // Also ensure that SR-IOV is disabled.
        clear_flags32(addr, IGB_SRIOV, IGB_SRIOV_EN);
        set_flags32(addr, IGB_GPIE, IGB_GPIE_MMSIX);

        for (unsigned int rq = 0; rq < num_rx_queues; rq++) {
            unsigned int      msi_vec;     // Vector assigned to this RX queue
            l4_icu_msi_info_t msi_info;

            // Get the assigned IRQ vector from the IVAR reg, see also sections
            // 7.3.2 and 8.8.15 of the I350 programmers manual
            ivar = get_reg32(addr, IGB_IVAR0 + 4 * (rq / 2));

            if ((rq % 2) == 0)
                msi_vec = ivar & 0x0000001f;
            else
                msi_vec = ivar & 0x001f0000;

            // NIC only allows for MSI-X vectors between 0 and 24
            if (msi_vec > 24)
                ixl_error("Obtained bad MSI-X vector %u from IVAR.", msi_vec);

            // Create and bind the IRQ to the vICU of the PCI device's bus
            create_and_bind_irq(msi_vec, &interrupts.queues[rq].irq,
                                interrupts.vicu);

            // Get the MSI info
            uint64_t source = pci_dev.dev_handle() | L4vbus::Icu::Src_dev_handle;
            L4Re::chksys(interrupts.vicu->msi_info(rq, source, &msi_info),
                         "Failed to retrieve MSI info.");
            ixl_debug("MSI info: vector = 0x%x addr = %llx, data = %x\n",
                      msi_vec, msi_info.msi_addr, msi_info.msi_data);

            // PCI-enable of MSI-X
            pcidev_enable_msix(msi_vec, msi_info, addr, table_offs, table_size);

            L4Re::chksys(l4_ipc_error(interrupts.vicu->unmask(rq), l4_utcb()),
                         "Failed to unmask interrupt");
            ixl_debug("Attached to MSI-X %u", msi_vec);

            interrupts.queues[rq].moving_avg.length = 0;
            interrupts.queues[rq].moving_avg.index  = 0;
            interrupts.queues[rq].msi_vec           = msi_vec;
            interrupts.queues[rq].interval = INTERRUPT_INITIAL_INTERVAL;
        }
    }
    else {
        // Disable IRQs completely if MSI-X is not present
        interrupts.interrupt_type = IXL_IRQ_LEGACY;

        ixl_warn("Device does not support MSIs. Disabling interrupts...");
        interrupts.interrupts_enabled = false;
        return;
    }

}

void Igb_device::init_link(void) {
    ixl_error("Unimplemented for Igb.");
    return;
}

void Igb_device::start_rx_queue(int queue_id) {
    ixl_debug("starting rx queue %d", queue_id);
    struct igb_rx_queue* queue = ((struct igb_rx_queue*) rx_queues) + queue_id;

    // Allocate packet buffers and set backing memory for descriptors
    // 2048 as pktbuf size is strictly speaking incorrect:
    // we need a few headers (1 cacheline), so there's only 1984 bytes
    // left for the device but the 82599 can only handle sizes in increments
    // of 1 kb; but this is fine since our max packet size is the default
    // MTU of 1518 this has to be fixed if jumbo frames are to be supported
    // mempool should be >= the number of rx and tx descriptors for a
    // forwarding application
    int mempool_size = NUM_RX_QUEUE_ENTRIES + NUM_TX_QUEUE_ENTRIES;
    if (mempool_size < MIN_MEMPOOL_ENTRIES)
        mempool_size = MIN_MEMPOOL_ENTRIES;

    queue->mempool = memory_allocate_mempool(*this, mempool_size,
                                             PKT_BUF_ENTRY_SIZE);
    if (queue->num_entries & (queue->num_entries - 1)) {
        ixl_error("number of queue entries must be a power of 2");
    }
    for (int j = 0; j < queue->num_entries; j++) {
        volatile struct igb_rx_desc* rxd = queue->descriptors + j;
        struct pkt_buf* buf = pkt_buf_alloc(queue->mempool);
        if (!buf) {
                ixl_error("failed to allocate rx buffer");
        }

        // Set buffer address and clear RXD flags
        rxd->buf_addr = buf->buf_addr_phy + offsetof(struct pkt_buf, data);
        rxd->errors   = 0;
        rxd->status   = 0;
        // we need to return the virtual address in the rx function
        // which the descriptor doesn't know by default
        queue->virtual_addresses[j] = buf;
    }

    ixl_debug("SRRCTL0 = %x", get_reg32(addr, IGB_SRRCTL0));

    // Wait for the enable bit to show up
    // FIXME: Implement this for other queues as well
    set_flags32(addr, IGB_RXDCTL0, IGB_RXDCTL_EN);
    wait_set_reg32(addr, IGB_RXDCTL0, IGB_RXDCTL_EN);

    // Only now set the final head and tail pointers (were initialized to 0)
    set_reg32(addr, IGB_RDH, 0);
    set_reg32(addr, IGB_RDT, queue->num_entries - 1);
}

/* Kicks of the TX part by configuring the TCTL register accordingly        */
void Igb_device::start_tx_queue(int queue_id) {
    (void) queue_id;

    // Enable queue 0
    // FIXME: As an optimization, we could also think about enabling bursted
    //        write back of finished send descriptors here.
    set_flags32(addr, IGB_TXDCTL0, IGB_TXDCTL_EN);

    uint32_t tctl = get_reg32(addr, IGB_TCTL);
    
    // Clear collision threshold bitmask
    tctl &= ~IGB_TCTL_CT;

    // Set collision threshold default values (16, as demanded by IEEE).
    tctl |= IGB_COLLISION_THRESHOLD << IGB_CT_SHIFT;

    // Enable TX queue, enable padding of short packets
    set_reg32(addr, IGB_TCTL, tctl | IGB_TCTL_EN | IGB_TCTL_PSP);
}

void Igb_device::init_rx(void) {
    // For now we assume that this function is only called immediately after
    // a reset operation with RX and TX disabled, so we do not need to 
    // disable them again here.

    // Disable VLAN filtering as we do not support it anyways
    clear_flags32(addr, IGB_RCTL, IGB_RCTL_VFE);

    // Actually, E1000 only has a single qp, but we leave the loop anyways
    for (uint16_t i = 0; i < num_rx_queues; i++) {
        ixl_debug("initializing rx queue %d", i);
       
        // Instruct NIC to drop packets if no RX descriptors are available
        // FIXME: When using multiple RX queues, choose the rights SRRCTL reg
        set_flags32(addr, IGB_SRRCTL0, IGB_SRRCTL_DREN);

        struct igb_rx_queue* queue = ((struct igb_rx_queue*) rx_queues) + i;
        
        uint32_t ring_size_bytes = NUM_RX_QUEUE_ENTRIES * sizeof(struct igb_rx_desc);
        struct dma_memory mem = memory_allocate_dma(*this, ring_size_bytes);
        
        // neat trick from Snabb: initialize to 0xFF to prevent rogue memory
        // accesses on premature DMA activation
        memset(mem.virt, -1, ring_size_bytes);

        // Keep a reference to mem in the queue, otherwise the object will go
        // out of scope, leading to the revocation of the backing capability
        queue->descr_mem = mem;

        // private data for the driver, 0-initialized
        queue->num_entries = NUM_RX_QUEUE_ENTRIES;
        queue->rx_index    = 0;
        queue->descriptors = (struct igb_rx_desc*) mem.virt;

        // Tell the device where it can write to (its iova, so DMA addrs)
        set_reg32(addr, IGB_RDBAL, (uint32_t) (mem.phy & 0xFFFFFFFFull));
        set_reg32(addr, IGB_RDBAH, (uint32_t) (mem.phy >> 32));
        set_reg32(addr, IGB_RDLEN, ring_size_bytes);
        // Set ring to empty at start
        set_reg32(addr, IGB_RDH, 0);
        set_reg32(addr, IGB_RDT, 0);
        ixl_debug("rx ring %d phy addr:  0x%012llX", i, mem.phy);
        ixl_debug("rx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);
    }

    // Enable checksum offloading for CRC and received UCP/TCP packets
    set_flags32(addr, IGB_RXCSUM, IGB_RXCSUM_TUOFL | IGB_RXCSUM_CRCOFL);

    // Merely kicks of the RX part by setting the RX enabled bit in RCTL
    // Also enables reception of broadcast frames (BAM)
    set_flags32(addr, IGB_RCTL, IGB_RCTL_EN | IGB_RCTL_BAM);
}

void Igb_device::init_tx(void) {
    // FIXME: Enable mq support for this type of NIC
    // Actually, E1000 only has a single qp, but we leave the loop anyways
    for (uint16_t i = 0; i < num_tx_queues; i++) {
        struct igb_tx_queue* queue = ((struct igb_tx_queue*) tx_queues) + i;
        ixl_debug("initializing tx queue %d", i);

        // setup descriptor ring, see section 7.1.9
        uint32_t ring_size_bytes = NUM_TX_QUEUE_ENTRIES * sizeof(struct igb_tx_desc);
        struct dma_memory mem = memory_allocate_dma(*this, ring_size_bytes);
        memset(mem.virt, -1, ring_size_bytes);

        // tell the device where it can write to (its iova, so DMA addrs)
        set_reg32(addr, IGB_TDBAL, (uint32_t) (mem.phy & 0xFFFFFFFFull));
        set_reg32(addr, IGB_TDBAH, (uint32_t) (mem.phy >> 32));
        set_reg32(addr, IGB_TDLEN, ring_size_bytes);
        ixl_debug("tx ring %d phy addr:  0x%012llX", i, mem.phy);
        ixl_debug("tx ring %d virt addr: 0x%012lX", i, (uintptr_t) mem.virt);

        // Init TX queue to empty
        set_reg32(addr, IGB_TDH, 0);
        set_reg32(addr, IGB_TDT, 0);

        // Keep a reference to mem in the queue, otherwise the object will go
        // out of scope, leading to the revocation of the backing capability
        queue->descr_mem   = mem;

        // private data for the driver, 0-initialized
        queue->num_entries = NUM_TX_QUEUE_ENTRIES;
        queue->descriptors = (struct igb_tx_desc*) mem.virt;
    }
}

void Igb_device::wait_for_link(void) {
    ixl_info("Waiting for link...");
    int32_t max_wait       = 10000000; // 10 seconds in us
    uint32_t poll_interval = 100000;   // 10 ms in us
    
    while (!get_link_speed() && max_wait > 0) {
        usleep(poll_interval);
        max_wait -= poll_interval;
    }

    ixl_info("Link speed is %d Mbit/s", get_link_speed());
    if (get_link_speed() == 0)
        ixl_error("Failed to bring up link!");
}

/* Reset the device and bring up the link again in a fresh state            */
void Igb_device::reset_and_init(void) {
    ixl_info("Resetting E1000 device.");

    // Prevent device from sending any more IRQs
    disable_interrupts();

    // Stop receive and transmit units, wait for pending transactions to
    // complete
    clear_flags32(addr, IGB_RCTL, IGB_RCTL_EN);
    clear_flags32(addr, IGB_TCTL, IGB_TCTL_EN);
    usleep(10000);

    // Issue the reset command (done by setting the reset bit in the ctrl reg)
    uint32_t ctrl_reg = get_reg32(addr, IGB_CTRL);
    
    // Set automatic link speed detection and force link to come up
    ctrl_reg |= IGB_CTRL_SLU;
    ctrl_reg &= ~(IGB_CTRL_FRCSPD | IGB_CTRL_FRCDPX);
    set_reg32(addr, IGB_CTRL, ctrl_reg | IGB_CTRL_PRT_RST | 
                                         IGB_CTRL_DEV_RST |
                                         IGB_CTRL_PHY_RST);
    // Wait for NIC to read default settings from EEPROM
    usleep(10000);

    // Once again, disable IRQs and clear pending interrupts
    disable_interrupts();
    clear_interrupts();

    ixl_info("Reset completed, starting init phase.");

    // Get the NIC's MAC address
    (void) get_mac_addr();

    ixl_info("Using MAC address %02x:%02x:%02x:%02x:%02x:%02x", 
             mac_addr.addr[0], mac_addr.addr[1], mac_addr.addr[2],
             mac_addr.addr[3], mac_addr.addr[4], mac_addr.addr[5]);

    ixl_debug("Programming MAC address into RAR[0]");
    uint32_t rar_lo = ((uint32_t) mac_addr.addr[0] |
                       ((uint32_t) mac_addr.addr[1] << 8) |
                       ((uint32_t) mac_addr.addr[2] << 16) |
                       ((uint32_t) mac_addr.addr[3] << 24));
    uint32_t rar_hi = ((uint32_t) mac_addr.addr[4] |
                       ((uint32_t) mac_addr.addr[5] << 8));
    // Mark RAR address entry valid
    rar_hi |= IGB_RAH_AV;

    set_reg32(addr, IGB_RA, rar_lo);
    set_reg32(addr, IGB_RA + 4, rar_hi);

    // Normally, we should clear the rest of the RAR table here. Let's see
    // whether we can skip this for now...

    // reset statistic registers
    read_stats(NULL);

    // Initialize receive and transmit data structures
    init_rx();
    init_tx();

    // Start RX and TX queues
    start_rx_queue(0);
    start_tx_queue(0);
    usleep(1000);

    // Enable IRQ for receiving packets
    if (interrupts.interrupts_enabled) {
        // Enable auto-clearing for all possible (25) MSIs
        set_reg32(addr, IGB_EIAC, 0x01ffffff);

        // For now, only enable the IRQ of the first receive queue
        enable_rx_interrupt(0);
    }

    // Enable promiscuous mode by default (facilitates testing)
    set_promisc(true);

    // Wait for the link to come up
    wait_for_link();
}

uint32_t Igb_device::rx_batch(uint16_t queue_id, struct pkt_buf* bufs[], 
                              uint32_t num_bufs) {
    struct interrupt_queue* interrupt = NULL;
    bool interrupts_enabled = interrupts.interrupts_enabled;

    // For non-debug builds insert an additional bounds check for the queue_id
    l4_assert(queue_id == 0);

    if (interrupts_enabled) {
        interrupt = &interrupts.queues[queue_id];
    }

    if (interrupts_enabled && interrupt->interrupt_enabled) {
        // We only listen on IRQs caused by this RQ, so nothing to do
        // afterwards as auto clearing is enabled
        ixl_debug("waiting for irq");
        interrupt->irq->receive(interrupts.timeout);
    }

    struct igb_rx_queue* queue = ((struct igb_rx_queue*) rx_queues) + queue_id;

    // rx index we checked in the last run of this function
    uint16_t rx_index = queue->rx_index;
    // index of the descriptor we checked in the last iteration of the loop
    uint16_t last_rx_index = rx_index;
    uint32_t buf_index;

    for (buf_index = 0; buf_index < num_bufs; buf_index++) {
        // rx descriptors are explained in section 3.2.3
        volatile struct igb_rx_desc* desc_ptr = queue->descriptors + rx_index;
        uint8_t status = desc_ptr->status;
        if (status & IGB_RXD_STAT_DD) {
            if (!(status & IGB_RXD_STAT_EOP)) {
                ixl_error("multi-segment packets are not supported - "
                          "increase buffer size or decrease MTU");
            }

            // got a packet, read and copy the whole descriptor
            struct igb_rx_desc desc;
            memcpy(&desc, (const void *) desc_ptr, sizeof(desc));
            struct pkt_buf* buf = (struct pkt_buf*) queue->virtual_addresses[rx_index];
            buf->size = desc.length;
            // this would be the place to implement RX offloading by translating
            // the device-specific flags to an independent representation in
            // the buf (similiar to how DPDK works)
            // need a new mbuf for the descriptor
            struct pkt_buf* new_buf = pkt_buf_alloc(queue->mempool);
            if (!new_buf) {
                // we could handle empty mempools more gracefully here, but it would be quite messy...
                // make your mempools large enough
                ixl_error("failed to allocate new mbuf for rx, you are either "
                          "leaking memory or your mempool is too small");
            }

            // reset the descriptor (new buffer address and zero out flags)
            // TODO: Is the zeroing of flags actually necessary for an E1000?
            desc_ptr->buf_addr = new_buf->buf_addr_phy + offsetof(struct pkt_buf, data);
            desc_ptr->errors   = 0;
            desc_ptr->status   = 0;

            queue->virtual_addresses[rx_index] = new_buf;
            bufs[buf_index] = buf;
            
            // want to read the next one in the next iteration, 
            // but we still need the last/current to update RDT later
            last_rx_index = rx_index;
            rx_index = wrap_ring(rx_index, queue->num_entries);
        } 
        else {
            break;
        }
    }
    if (rx_index != last_rx_index) {
        // tell hardware that we are done
        // this is intentionally off by one, otherwise we'd set RDT=RDH if 
        // we are receiving faster than packets are coming in
        // RDT=RDH means queue is full
        set_reg32(addr, IGB_RDT, last_rx_index);
        queue->rx_index = rx_index;
    }

    // Perform IRQ bookkeeping
    if (interrupts_enabled) {
        interrupt->rx_pkts += buf_index;

        if ((interrupt->instr_counter++ & 0xFFF) == 0) {
            bool int_en = interrupt->interrupt_enabled;
            uint64_t diff = device_stats::monotonic_time() - interrupt->last_time_checked;
            if (diff > interrupt->interval) {
                // every second
                check_interrupt(interrupt, diff, buf_index, num_bufs);
            }

            if (int_en != interrupt->interrupt_enabled) {
                if (interrupt->interrupt_enabled) {
                    enable_rx_interrupt(queue_id);
                } else {
                    disable_rx_interrupt(queue_id);
                }
            }
        }
    }

    // number of packets stored in bufs; buf_index points to the next index
    return buf_index;
}

uint32_t Igb_device::tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                              uint32_t num_bufs) {
    struct igb_tx_queue* queue = ((struct igb_tx_queue*) tx_queues) + queue_id;
    // the descriptor is explained in section 3.3.2
    // we just use a struct copy & pasted from intel, but it basically has two formats (hence a union):
    // 1. the write-back format which is written by the NIC once sending it is finished this is used in step 1
    // 2. the read format which is read by the NIC and written by us, this is used in step 2

    uint16_t clean_index = queue->clean_index; // next descriptor to clean up

    // step 1: clean up descriptors that were sent out by the hardware and return them to the mempool
    // start by reading step 2 which is done first for each packet
    // cleaning up must be done in batches for performance reasons, so this is unfortunately somewhat complicated
    while (true) {
        // figure out how many descriptors can be cleaned up

        // tx_index is always ahead of clean (invariant of our queue)
        int32_t cleanable = queue->tx_index - clean_index;
        if (cleanable < 0) { 
            // handle wrap-around
            cleanable = queue->num_entries + cleanable;
        }
        if (cleanable < TX_CLEAN_BATCH) {
            break;
        }
        // calculcate the index of the last transcriptor in the clean batch
        // we can't check all descriptors for performance reasons
        int32_t cleanup_to = clean_index + TX_CLEAN_BATCH - 1;
        if (cleanup_to >= queue->num_entries) {
            cleanup_to -= queue->num_entries;
        }
        volatile struct igb_tx_desc* txd = queue->descriptors + cleanup_to;
        // hardware sets this flag as soon as it's sent out, we can give back all bufs in the batch back to the mempool
        if (txd->upper.data & IGB_TXD_STAT_DD) {
            int32_t i = clean_index;
            while (true) {
                struct pkt_buf* buf = (struct pkt_buf *) queue->virtual_addresses[i];
                pkt_buf_free(buf);
                if (i == cleanup_to) {
                    break;
                }
                i = wrap_ring(i, queue->num_entries);
            }
            // next descriptor to be cleaned up is one after the one we just cleaned
            clean_index = wrap_ring(cleanup_to, queue->num_entries);
        } else {
            // clean the whole batch or nothing; yes, this leaves some packets in
            // the queue forever if you stop transmitting, but that's not a real concern
            break;
        }
    }
    queue->clean_index = clean_index;

    // step 2: send out as many of our packets as possible
    uint32_t sent;
    for (sent = 0; sent < num_bufs; sent++) {
        uint32_t next_index = wrap_ring(queue->tx_index, queue->num_entries);
        // we are full if the next index is the one we are trying to reclaim
        if (clean_index == next_index) {
            break;
        }
        struct pkt_buf* buf = bufs[sent];
        // remember virtual address to clean it up later
        queue->virtual_addresses[queue->tx_index] = (void*) buf;
        volatile struct igb_tx_desc* txd = queue->descriptors + queue->tx_index;
        queue->tx_index = next_index;
        // NIC reads from here
        txd->buffer_addr  = buf->buf_addr_phy + offsetof(struct pkt_buf, data);

        // Reset descriptor command before sending (otherwise NIC won't emit
        // any data!)
        txd->lower.data          = 0;
        txd->lower.flags.length  = buf->size;
        txd->upper.fields.status = 0;

        // always the same flags: one buffer (EOP), CRC offload, report status
        txd->lower.data |= IGB_TXD_CMD_EOP | IGB_TXD_CMD_IFCS |
                           IGB_TXD_CMD_RS;
        // no fancy offloading stuff - only the total payload length
        // implement offloading flags here:
        //      * ip checksum offloading is trivial: just set the offset
        //      * tcp/udp checksum offloading is more annoying, 
        //        you have to precalculate the pseudo-header checksum
        // TODO: Implement TCP / UDP offloading here...
    }

    // send out by advancing tail, i.e., pass control of the bufs to the nic
    // this seems like a textbook case for a release memory order,
    // but Intel's driver doesn't even use a compiler barrier here
    set_reg32(addr, IGB_TDT, queue->tx_index);
    return sent;
}

/* Read a subset of the NIC's statistic registers (see section 13.7)        */
void Igb_device::read_stats(struct device_stats *stats) {
    // Keep in mind that reading the counters will reset them
    uint32_t rx_pkts = get_reg32(addr, IGB_GPRC);
    uint32_t tx_pkts = get_reg32(addr, IGB_GPTC);
    // Lower reg. resets when higher reg is read
    uint64_t rx_bytes = get_reg32(addr, IGB_GORCL) +
                        (((uint64_t) get_reg32(addr, IGB_GORCH)) << 32);
    uint64_t tx_bytes = get_reg32(addr, IGB_GOTCL) +
                        (((uint64_t) get_reg32(addr, IGB_GOTCH)) << 32);
    
    // Sum up the counters if a stat object was given
    if (stats != NULL) {
        stats->rx_pkts  += rx_pkts;
        stats->tx_pkts  += tx_pkts;
        stats->rx_bytes += rx_bytes;
        stats->tx_bytes += tx_bytes;
    }
}

void Igb_device::set_promisc(bool enabled) {
    // Set / clear settings for both unicast and multicast packets
    if (enabled) {
        ixl_info("enabling promisc mode");
        set_flags32(addr, IGB_RCTL, IGB_RCTL_MPE | IGB_RCTL_UPE);
    } 
    else {
        ixl_info("disabling promisc mode");
        clear_flags32(addr, IGB_RCTL, IGB_RCTL_MPE | IGB_RCTL_UPE);
    }
}

/* Get the link speed in Mbps, or 0 if link is down                         */
uint32_t Igb_device::get_link_speed(void) {
    uint32_t status = get_reg32(addr, IGB_STATUS);

    if (!(status & IGB_STATUS_LU)) {
        return 0;
    }
    switch (status & IGB_STATUS_SPEED_MASK) {
        case IGB_STATUS_SPEED_10:
            return 10;
        case IGB_STATUS_SPEED_100:
            return 100;
        case IGB_STATUS_SPEED_1000:
            return 1000;
        default:
            // Hm, data sheet says that 0x000000c0 also indicates 1 Gbps, but
            // the Linux kernel ignores this...
            return 0;
    }
}

struct mac_address Igb_device::get_mac_addr(void) {
    // Check whether there is a valid MAC address for this device. If not,
    // read it from EEPROM
    if (! mac_init) {
        if (read_eeprom(0x0, 3, (uint16_t *) &mac_addr.addr) != 0)
            ixl_error("Failed to read MAC address from EEPROM.");
    
        mac_init = true;
    }

    return mac_addr;
}

void Igb_device::set_mac_addr(struct mac_address mac) {
    (void) mac;
    ixl_error("Unimplemented.");
    return;
}

Igb_device* Igb_device::igb_init(L4vbus::Pci_dev&& pci_dev,
                                 uint16_t rx_queues,
                                 uint16_t tx_queues,
                                 int irq_timeout) {

    // Allocate memory for the ixgbe device that will be returned               
    // TODO: Check whether these IRQ settings are meaningful for Igb.
    Igb_device *dev = new Igb_device(std::move(pci_dev),          
                                     rx_queues, tx_queues,                  
                                     (irq_timeout != 0),                    
                                     0x028, // itr_rate (10ys => 97600 INT/s)
                                     irq_timeout);                          
                                                                                
    dev->reset_and_init();                                                      
    return dev;  
}
