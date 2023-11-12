#ifndef IXY_INTERRUPTS_H
#define IXY_INTERRUPTS_H

#include <stdint.h>

#include <l4/sys/irq>

#define MOVING_AVERAGE_RANGE 5
#define INTERRUPT_THRESHOLD 1200

namespace Ixl {

const int IXL_IRQ_MSIX   = 0x02;
const int IXL_IRQ_MSI    = 0x01;
const int IXL_IRQ_LEGACY = 0x00;

struct interrupt_moving_avg {
    uint32_t index; // The current index
    uint32_t length; // The moving average length
    uint64_t sum; // The moving average sum
    uint64_t measured_rates[MOVING_AVERAGE_RANGE]; // The moving average window
};

struct interrupt_queue {
    L4::Cap<L4::Irq> irq;       // Cap to L4 IRQ object

    int vfio_event_fd;          // event fd
    int vfio_epoll_fd;          // epoll fd
    bool interrupt_enabled;     // Whether interrupt for this queue is enabled or not
    uint64_t last_time_checked; // Last time the interrupt flag was checked
    uint64_t instr_counter;     // Instruction counter to avoid unnecessary calls to monotonic_time
    uint64_t rx_pkts;           // The number of received packets since the last check
    uint64_t interval;          // The interval to check the interrupt flag
    struct interrupt_moving_avg moving_avg; // The moving average of the hybrid interrupt
};

struct interrupts {
    // Whether interrupts for this device are enabled or disabled.
    bool interrupts_enabled;

    // Cap to the virtual interrupt controller of the ixylon device
    L4::Cap<L4::Icu> vicu;

    uint32_t itr_rate;               // The Interrupt Throttling Rate
    struct interrupt_queue* queues; // Interrupt settings per queue
    uint8_t interrupt_type;          // MSI or MSIX

    // interrupt timeout in L4-specific representation
    l4_timeout_t timeout;
};

/**
 * Check if interrupts or polling should be used based on the current number
 * of received packets per seconds.
 *
 * \param interrupts The interrupt handler.
 * \param diff The difference since the last call in nanoseconds.
 * \param buf_index The current buffer index.
 * \param buf_size The maximum buffer size.
 *
 * \return Whether to disable NIC interrupts or not.
 */
void check_interrupt(struct interrupt_queue* interrupt, uint64_t diff, uint32_t buf_index, uint32_t buf_size);

/**
 * Allocates and IRQ cap into the object provided by the client and subsequently
 * binds this IRQ to the ICU given.
 *
 * \param irqnum IRQ number that shall be used during bind operation.
 * \param irq    IRQ object to fill with a new capability. Must not be NULL!
 * \param icu    IRQ control unit that irq shall be bound to.
 */
void create_and_bind_irq(unsigned int irqnum, L4::Cap<L4::Irq> *irq,
                         L4::Cap<L4::Icu> icu);

} // namespace Ixl

#endif //IXY_INTERRUPTS_H
