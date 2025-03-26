#include <l4/ixl/stats.h>

#include <stdio.h>

using namespace Ixl;

// initializes a stat struct and clears the stats on the device
device_stats::device_stats(Ixl_device *dev) : device(dev) {
    // might require device-specific initialization
    rx_pkts   = 0;
    tx_pkts   = 0;
    rx_bytes  = 0;
    tx_bytes  = 0;
    device->read_stats(NULL);
}

void device_stats::print_stats(struct device_stats* stats) {
    printf("RX: %zu bytes %zu packets\n", stats->rx_bytes, stats->rx_pkts);
    printf("TX: %zu bytes %zu packets\n", stats->tx_bytes, stats->tx_pkts);
}

static double diff_mpps(uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
    return (double) (pkts_new - pkts_old) / 1000000.0 / ((double) nanos / 1000000000.0);
}

static uint32_t diff_mbit(uint64_t bytes_new, uint64_t bytes_old, uint64_t pkts_new, uint64_t pkts_old, uint64_t nanos) {
    // take stuff on the wire into account, i.e., the preamble, SFD and IFG (20 bytes)
    // otherwise it won't show up as 10000 mbit/s with small packets which is confusing
    return (uint32_t) (((bytes_new - bytes_old) / 1000000.0 / ((double) nanos / 1000000000.0)) * 8
        + diff_mpps(pkts_new, pkts_old, nanos) * 20 * 8);
}

void device_stats::print_stats_diff(struct device_stats* stats_new, struct device_stats* stats_old, uint64_t nanos) {
    printf("RX: %d Mbit/s %.2f Mpps\n", diff_mbit(stats_new->rx_bytes,
        stats_old->rx_bytes, stats_new->rx_pkts, stats_old->rx_pkts, nanos),
        diff_mpps(stats_new->rx_pkts, stats_old->rx_pkts, nanos)
    );
    printf("TX: %d Mbit/s %.2f Mpps\n", diff_mbit(stats_new->tx_bytes,
        stats_old->tx_bytes, stats_new->tx_pkts, stats_old->tx_pkts, nanos),
        diff_mpps(stats_new->tx_pkts, stats_old->tx_pkts, nanos)
    );
}


// returns a timestamp in nanoseconds
// based on rdtsc on reasonably configured systems and is hence fast
uint64_t device_stats::monotonic_time() {
    struct timespec timespec;
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    return timespec.tv_sec * 1000 * 1000 * 1000 + timespec.tv_nsec;
}
