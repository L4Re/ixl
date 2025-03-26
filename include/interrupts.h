/*****************************************************************************
 *                                                                           *
 * Interrupts - IRQ-specific funtions used by the Ixl driver framework.      *
 *                                                                           *
 * Some parts of this header still originate from the Ixy project, while the *
 * majority has been rewritten in C++ and was adapted to L4Re.               *
 *                                                                           *
 *****************************************************************************/

#pragma once

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

    bool interrupt_enabled;     // Whether interrupt for this queue is enabled or not
    uint32_t msi_vec;           // MSI vector used for this IRQ (or 0 for legacy)
    uint64_t last_time_checked; // Last time the interrupt flag was checked
    uint64_t instr_counter;     // Instruction counter to avoid unnecessary calls to monotonic_time
    uint64_t rx_pkts;           // The number of received packets since the last check
    uint64_t interval;          // The interval to check the interrupt flag
    struct interrupt_moving_avg moving_avg; // The moving average of the hybrid interrupt
};

enum class interrupt_mode {
    // RX interrupts are disabled for this device.
    Disable,
    // RX interrupts are enabled for this device. They are used in rx_batch()
    // when waiting for new packets to arrive.
    Wait,
    // RX interrupts are enabled for this device. They are used as an
    // asynchronous notification when new packets arrive.
    Notify,
};

struct interrupts {
    // Whether and how interrupts are used for this device.
    interrupt_mode mode;

    // Cap to the virtual interrupt controller of the Ixl device
    L4::Cap<L4::Icu> vicu;

    uint32_t itr_rate;               // The Interrupt Throttling Rate
    struct interrupt_queue* queues;  // Interrupt settings per queue
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
 * NOTE: If you intend to bind irq to an MSI interrupt, make sure to OR the
 *       irqnum with L4::Icu::F_msi before passing it to this function, as
 *       imposed by the L4Re API.
 *
 * \param irqnum IRQ number that shall be used during bind operation.
 * \param irq    IRQ object to fill with a new capability. Must not be NULL!
 * \param icu    IRQ control unit that irq shall be bound to.
 */
void create_and_bind_irq(unsigned int irqnum, L4::Cap<L4::Irq> *irq,
                         L4::Cap<L4::Icu> icu);

} // namespace Ixl
