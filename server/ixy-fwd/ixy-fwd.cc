#include <stdio.h>

#include <l4/re/error_helper>
#include <l4/re/env>

#include <l4/ixylon/stats.h>
#include <l4/ixylon/memory.h>
#include <l4/ixylon/device.h>

const int BATCH_SIZE = 32;

static void forward(Ixl::Ixl_device* rx_dev, uint16_t rx_queue, Ixl::Ixl_device* tx_dev, uint16_t tx_queue) {
	struct Ixl::pkt_buf* bufs[BATCH_SIZE];
	uint32_t num_rx = rx_dev->rx_batch(rx_queue, bufs, BATCH_SIZE);
	if (num_rx > 0) {
		// touch all packets, otherwise it's a completely unrealistic workload if the packet just stays in L3
		for (uint32_t i = 0; i < num_rx; i++) {
			bufs[i]->data[1]++;
		}
		uint32_t num_tx = tx_dev->tx_batch(tx_queue, bufs, num_rx);
		// there are two ways to handle the case that packets are not being sent out:
		// either wait on tx or drop them; in this case it's better to drop them, otherwise we accumulate latency
		for (uint32_t i = num_tx; i < num_rx; i++) {
			Ixl::pkt_buf_free(bufs[i]);
		}
	}
}

int main(int argc, char* argv[]) {
	if (argc != 3) {
		printf("%s forwards packets between two ports.\n", argv[0]);
		printf("Usage: %s <vbus dev idx 2> <vbus dev idx 1>\n", argv[0]);
		return 1;
	}

    // Get vbus capability received on startup                                  
    auto vbus = L4Re::chkcap(L4Re::Env::env()->get_cap<L4vbus::Vbus>("vbus"),   
                             "Get vbus capability.", -L4_ENOENT); 

	Ixl::Ixl_device* dev1 = Ixl::Ixl_device::ixl_init(vbus, 
                                                      atoi(argv[1]), 1, 1, -1);
	Ixl::Ixl_device* dev2 = Ixl::Ixl_device::ixl_init(vbus,
                                                      atoi(argv[2]), 1, 1, 0);

	uint64_t last_stats_printed = Ixl::device_stats::monotonic_time();
	Ixl::device_stats stats1(dev1);
	Ixl::device_stats stats1_old(dev1);
	Ixl::device_stats stats2(dev2);
	Ixl::device_stats stats2_old(dev2);

	uint64_t counter = 0;
	while (true) {
		forward(dev1, 0, dev2, 0);
		forward(dev2, 0, dev1, 0);

		// don't poll the time unnecessarily
		if ((counter++ & 0xFFF) == 0) {
			uint64_t time = Ixl::device_stats::monotonic_time();
			if (time - last_stats_printed > 1000 * 1000 * 1000) {
				// every second
				dev1->read_stats(&stats1);
				Ixl::device_stats::print_stats_diff(&stats1, &stats1_old, time - last_stats_printed);
				stats1_old = stats1;
				if (dev1 != dev2) {
					dev2->read_stats(&stats2);
					Ixl::device_stats::print_stats_diff(&stats2, &stats2_old, time - last_stats_printed);
					stats2_old = stats2;
				}
				last_stats_printed = time;
			}
		}
	}
}

