#ifndef IXY_STATS_H
#define IXY_STATS_H

#include <stdint.h>
#include <stddef.h>
#include <time.h>
#include "device.h"

namespace Ixl {

struct device_stats {

    device_stats(Ixl_device *dev);

    static void print_stats(struct device_stats* stats);
    
    static void print_stats_diff(struct device_stats* stats_new,
                                 struct device_stats* stats_old,
                                 uint64_t nanos_passed);

    static uint64_t monotonic_time();

    Ixl_device *device;
    size_t rx_pkts;
    size_t tx_pkts;
    size_t rx_bytes;
    size_t tx_bytes;
};

}

#endif //IXY_STATS_H
