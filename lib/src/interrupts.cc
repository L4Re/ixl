/****************************************************************************
 *
 * Implementation of Interrupt-related functions on L4Re.
 *
 * The ppms and check_interrupt functions are largely taken from the original
 * ixy driver.
 *
 * Copyright (C) 2023 Till Miemietz <till.miemietz@barkhauseninstitut.org>
 */

/****************************************************************************
 *                                                                          *
 *                           include statements                             *
 *                                                                          *
 ****************************************************************************/


#include <stdio.h>

#include <l4/re/error_helper>

#include <l4/ixylon/interrupts.h>
#include "libixy-vfio.h"
#include <l4/ixylon/log.h>
#include <l4/ixylon/stats.h>

/****************************************************************************
 *                                                                          *
 *                        function implementation                           *
 *                                                                          *
 ****************************************************************************/

using namespace Ixl;

/**
 * Calculate packets per millisecond based on the received number of packets and the elapsed time in nanoseconds since the
 * last calculation.
 * @param received_pkts Number of received packets.
 * @param elapsed_time_nanos Time elapsed in nanoseconds since the last calculation.
 * @return Packets per millisecond.
 */
static uint64_t ppms(uint64_t received_pkts, uint64_t elapsed_time_nanos) {
    return received_pkts / (elapsed_time_nanos / 1000000);
}

/*Check if interrupts or polling should be used                             */
void Ixl::check_interrupt(struct interrupt_queue* interrupt, uint64_t diff,
                          uint32_t buf_index, uint32_t buf_size) {
    struct interrupt_moving_avg* avg = &interrupt->moving_avg;
    avg->sum -= avg->measured_rates[avg->index];
    avg->measured_rates[avg->index] = ppms(interrupt->rx_pkts, diff);
    avg->sum += avg->measured_rates[avg->index];
    if (avg->length < MOVING_AVERAGE_RANGE) {
        avg->length++;
    }
    avg->index = (avg->index + 1) % MOVING_AVERAGE_RANGE;
    interrupt->rx_pkts = 0;
    uint64_t average = avg->sum / avg->length;
    if (average > INTERRUPT_THRESHOLD) {
        interrupt->interrupt_enabled = false;
    } else if (buf_index == buf_size) {
        interrupt->interrupt_enabled = false;
    } else {
        interrupt->interrupt_enabled = true;
    }
    interrupt->last_time_checked = Ixl::device_stats::monotonic_time();
}

/* Allocates an IRQ capability and binds it to an ICU.                      */
void Ixl::create_and_bind_irq(unsigned int irqnum, L4::Cap<L4::Irq> *irq,
                              L4::Cap<L4::Icu> icu) {
    *irq = L4Re::chkcap(L4Re::Util::cap_alloc.alloc<L4::Irq>(),
                        "Failed to allocated IRQ capability.");

    L4Re::chksys(l4_error(icu->bind(irqnum, *irq)),
                 "Binding interrupt to ICU.");
}
