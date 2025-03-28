/*****************************************************************************
 *                                                                           *
 *       ixl-pktgen - Send statically configured raw Ethernet frames         *
 *                                                                           *
 * This version of ixl-pktgen is an adapted form of the original ixy-pktgen  *
 * from the ixy driver package.                                              *
 *                                                                           *
 *****************************************************************************/


#include <stdio.h>

#include <l4/re/error_helper>
#include <l4/re/env>

#include <l4/ixl/stats.h>
#include <l4/ixl/log.h>
#include <l4/ixl/memory.h>
#include <l4/ixl/device.h>

using Ixl::Ixl_device;
using Ixl::device_stats;

// number of packets sent simultaneously to our driver
static const uint32_t BATCH_SIZE = 1;

// excluding CRC (offloaded by default)
#define PKT_SIZE 60

static const uint8_t pkt_data[] = {
    0x26, 0x69, 0x1e, 0xe7, 0xc6, 0x17, // dst MAC
    0x52, 0x54, 0x00, 0x12, 0x34, 0x56, // src MAC
    0x08, 0x00,                         // ether type: IPv4
    0x45, 0x00,                         // Version, IHL, TOS
    (PKT_SIZE - 14) >> 8,               // ip len excluding ethernet, high byte
    (PKT_SIZE - 14) & 0xFF,             // ip len exlucding ethernet, low byte
    0x00, 0x00, 0x00, 0x00,             // id, flags, fragmentation
    0x40, 0x11, 0x00, 0x00,             // TTL (64), protocol (UDP), checksum
    0x0A, 0x0A, 0x0A, 0x02,             // src ip (10.10.10.2)
    0x0A, 0x0A, 0x0A, 0x01,             // dst ip (10.10.10.1)
    0x1F, 0x90, 0x1F, 0x90,             // src and dst ports (8080 -> 8080)
    (PKT_SIZE - 20 - 14) >> 8,          // udp len excluding ip & ethernet, high byte
    (PKT_SIZE - 20 - 14) & 0xFF,        // udp len exlucding ip & ethernet, low byte
    0x00, 0x00,                         // udp checksum, optional
    'i', 'x', 'y'                       // payload
    // rest of the payload is zero-filled because mempools guarantee empty bufs
};

// calculate a IP/TCP/UDP checksum
static uint16_t calc_ip_checksum(uint8_t* data, uint32_t len) {
    if (len % 1) ixl_error("odd-sized checksums NYI"); // we don't need that
    uint32_t cs = 0;
    for (uint32_t i = 0; i < len / 2; i++) {
        cs += ((uint16_t*)data)[i];
        if (cs > 0xFFFF) {
            cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
        }
    }
    return ~((uint16_t) cs);
}

static struct Ixl::Mempool* init_mempool(Ixl_device& dev) {
    const int NUM_BUFS = 1024;
    struct Ixl::Mempool* mempool = new Ixl::Mempool(dev, NUM_BUFS, 0,
                                                    1ULL << 28);

    // pre-fill all our packet buffers with some templates that can be modified later
    // we have to do it like this because sending is async in the hardware; we cannot re-use a buffer immediately
    struct Ixl::pkt_buf* bufs[NUM_BUFS];
    for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
        struct Ixl::pkt_buf* buf = mempool->pkt_buf_alloc();
        buf->size = PKT_SIZE;
        memcpy(buf->data, pkt_data, sizeof(pkt_data));
        *(uint16_t*) (buf->data + 24) = calc_ip_checksum(buf->data + 14, 20);
        bufs[buf_id] = buf;
    }
    // return them all to the mempool, all future allocations will return bufs with the data set above
    for (int buf_id = 0; buf_id < NUM_BUFS; buf_id++) {
        Ixl::pkt_buf_free(bufs[buf_id]);
    }

    return mempool;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: %s <vbus dev idx>\n", argv[0]);
        return 1;
    }

    // Get vbus capability received on startup
    auto vbus = L4Re::chkcap(L4Re::Env::env()->get_cap<L4vbus::Vbus>("vbus"),
                             "Get vbus capability.", -L4_ENOENT);

    // Get a device configuration struct with default parameters
    struct Ixl::Dev_cfg cfg;
    // For this test, configure the device to use polling mode
    cfg.irq_timeout_ms = 0;

    // Create an Ixl device. This also initializes the NIC.
    Ixl_device* dev = Ixl_device::ixl_init(vbus, atoi(argv[1]), cfg);
    struct Ixl::Mempool* mempool = init_mempool(*dev);

    uint64_t last_stats_printed = device_stats::monotonic_time();
    uint64_t counter = 0;
    struct device_stats stats(dev);
    struct device_stats stats_old(dev);
    uint32_t seq_num = 0;

    // array of bufs sent out in a batch
    struct Ixl::pkt_buf* bufs[BATCH_SIZE];

    // tx loop
    while (true) {
        // we cannot immediately recycle packets, we need to allocate new packets every time
        // the old packets might still be used by the NIC: tx is async
        mempool->pkt_buf_alloc_batch(bufs, BATCH_SIZE);
        for (uint32_t i = 0; i < BATCH_SIZE; i++) {
            // packets can be modified here, make sure to update the checksum when changing the IP header
            *(uint32_t*)(bufs[i]->data + PKT_SIZE - 4) = seq_num++;
        }
        // the packets could be modified here to generate multiple flows
        dev->tx_batch_busy_wait(0, bufs, BATCH_SIZE);

        // don't check time for every packet, this yields +10% performance :)
        if ((counter++ & 0xFFF) == 0) {
            uint64_t time = device_stats::monotonic_time();
            if (time - last_stats_printed > 1000 * 1000 * 1000) {
                // every second
                dev->read_stats(&stats);
                device_stats::print_stats_diff(&stats, &stats_old, time - last_stats_printed);
                stats_old = stats;
                last_stats_printed = time;
            }
        }
        // track stats
        sleep(3);
    }
}

