/***************************************************************************** 
 *                                                                           * 
 * E1000_device - Implementation of a simple e1000 NIC device driver on L4.  * 
 *                                                                           * 
 * Copyright (C) 2023 Till Miemietz <till.miemietz@barkhauseninstitut.org>   * 
 *                                                                           * 
 *****************************************************************************/

#include "driver/e1000/e1000.h"

using namespace Ixl;

void E1000_device::setup_interrupts() {
    ixl_error("Interrupts are currently not supported.");
}

void E1000_device::init_link(void) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::start_rx_queue(int queue_id) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::start_tx_queue(int queue_id) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::init_rx(void) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::init_tx(void) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::wait_for_link(void) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::reset_and_init(void) {
    ixl_error("Unimplemented.");
    return;
}

uint32_t E1000_device::rx_batch(uint16_t queue_id, struct pkt_buf* bufs[], 
                                uint32_t num_bufs) {
    ixl_error("Unimplemented.");
    return 0;
}

uint32_t E1000_device::tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                                uint32_t num_bufs) {
    ixl_error("Unimplemented.");
    return 0;
}

void E1000_device::read_stats(struct device_stats *stats) {
    ixl_error("Unimplemented.");
    return;
}

void E1000_device::set_promisc(bool enabled) {
    ixl_error("Unimplemented.");
    return;
}

uint32_t E1000_device::get_link_speed(void) {
    ixl_error("Unimplemented.");
    return 0;
}

struct mac_address E1000_device::get_mac_addr(void) {
    ixl_error("Unimplemented.");
    struct mac_address addr;

    return addr;
}

void E1000_device::set_mac_addr(struct mac_address mac) {
    ixl_error("Unimplemented.");
    return;
}

E1000_device* E1000_device::e1000_init(const char *pci_addr,
                                       L4vbus::Pci_dev&& pci_dev,
                                       uint16_t rx_queues,
                                       uint16_t tx_queues,
                                       int irq_timeout) {

    // Allocate memory for the ixgbe device that will be returned               
    // TODO: Check whether these IRQ settings are meaningful for E1000.
    E1000_device *dev = new E1000_device(pci_addr, std::move(pci_dev),          
                                         rx_queues, tx_queues,                  
                                         (irq_timeout != 0),                    
                                         0x028, // itr_rate (10ys => 97600 INT/s)
                                         irq_timeout);                          
                                                                                
    dev->reset_and_init();                                                      
    return dev;  
}
