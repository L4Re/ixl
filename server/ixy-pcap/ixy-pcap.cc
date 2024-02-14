/*****************************************************************************
 *                                                                           *
 *    ixy-pcap - Dump raw ethernet frames received on a NIC to stderr        *
 *                                                                           *
 * This version of ixy-pcap is an adapted form of the original ixy-pcap of   *
 * the ixy driver package.                                                   *
 *                                                                           *
 *****************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

#include <l4/re/error_helper>
#include <l4/re/env>

#include <l4/ixylon/device.h>
#include <l4/ixylon/memory.h>
#include <l4/ixylon/log.h>

using Ixl::Ixl_device;

const int BATCH_SIZE = 32;

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        printf("Usage: %s <vbus dev idx> [n packets]\n", argv[0]);
        return 1;
    }

    // Get vbus capability received on startup
    auto vbus = L4Re::chkcap(L4Re::Env::env()->get_cap<L4vbus::Vbus>("vbus"),
                             "Get vbus capability.", -L4_ENOENT);

    Ixl_device* dev = Ixl_device::ixl_init(vbus, atoi(argv[1]), 1, 1, 0);

    int64_t n_packets = -1;
    if (argc == 3) {
        n_packets = atol(argv[2]);
    }

    if (n_packets >= 0) {
        printf("Capturing %ld packets...\n", n_packets);
    } else {
        printf("Capturing packets...\n");
    }

    struct Ixl::pkt_buf* bufs[BATCH_SIZE];
    while (n_packets != 0) {
        uint32_t num_rx = dev->rx_batch(0, bufs, BATCH_SIZE);
        struct timeval tv;
        gettimeofday(&tv, NULL);

        for (uint32_t i = 0; i < num_rx && n_packets != 0; i++) {

            fprintf(stderr, "===== Packet Received at %06ld:%06ld "
                    "(Payload length %u Bytes) =====\n",
                    tv.tv_sec, tv.tv_usec, bufs[i]->size);
            ixl_hexdump(bufs[i]->data, bufs[i]->size);
            fprintf(stderr, "=============================================="
                            "=========================\n");

            Ixl::pkt_buf_free(bufs[i]);
            // n_packets == -1 indicates unbounded capture
            if (n_packets > 0) {
                n_packets--;
            }
        }
    }

    printf("Capturing done...\n");
    return 0;
}
