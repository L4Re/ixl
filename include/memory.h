#ifndef IXY_MEMORY_H
#define IXY_MEMORY_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <atomic>

#include <l4/re/util/shared_cap>
#include <l4/re/dma_space>

#include <l4/ixylon/device.h>

namespace Ixl {

#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS) // 2_097_152 = 2MiB
#define SIZE_PKT_BUF_HEADROOM (40 - sizeof(std::atomic_uint32_t))

struct pkt_buf {
    // physical address to pass a buffer to a nic
    L4Re::Dma_space::Dma_addr buf_addr_phy;

    // Memory pool that this packet buffer belongs to
    struct mempool* mempool;
    // Index of this packet in the mempool
    uint32_t mempool_idx;

    // Amount of payload data in this packet
    uint32_t size;

    // Reference counter. Upper layers of a network stack may use this counter
    // for usage tracking of buffers, returning unused entries to the mempool
    // FIXME: Using a C++ atomic in this place is dangerous, as there is no
    //        guarantee on the size of this type. Hence, adding ref_cnt here
    //        could easily exceed the 64B size requirement of struct pkt_buf.
    //        However, adding the refcounter here is the cleaner than
    //        maintaining an additional "packet ref count map"...
    std::atomic_uint32_t ref_cnt;

    uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
    uint8_t data[] __attribute__((aligned(64)));
};

static_assert(sizeof(struct pkt_buf) == 64, "pkt_buf too large");
static_assert(offsetof(struct pkt_buf, data) == 64, "data at unexpected position");
static_assert(offsetof(struct pkt_buf, head_room) + SIZE_PKT_BUF_HEADROOM == offsetof(struct pkt_buf, data), "head room not immediately before data");

struct dma_memory {
    L4Re::Util::Shared_cap<L4Re::Dataspace> mem_cap;

    void* virt;
    L4Re::Dma_space::Dma_addr phy;
};

// everything here contains virtual addresses, the mapping to physical addresses are in the pkt_buf
struct mempool {
    // Backing DMA memory (DMA-enabled Dataspace)
    struct dma_memory backing_mem;

    void* base_addr;
    
    uint32_t buf_size;
    uint32_t num_entries;

    // Memory is managed with a lock-free queue for indicating which buffers
    // are currently available. Note that this queue is only a list of array
    // indices pointing to each other (no real "data objects" are stored)
    // which allows for a very compressed representation. UINT32_MAX serves as
    // representation for an invalid index (i.e., queue end).

    std::atomic_uint32_t queue_head;
    std::atomic_uint32_t queue_tail;

    // The queue contains the entry id, i.e., base_addr + entry_id * buf_size
    // is the address of the buf
    std::atomic_uint32_t free_queue[];
};

struct dma_memory memory_allocate_dma(Ixl_device& dev, size_t size);

struct mempool* memory_allocate_mempool(Ixl_device &dev,
                                        uint32_t num_entries,
                                        uint32_t entry_size);

uint32_t pkt_buf_alloc_batch(struct mempool* mempool, struct pkt_buf* bufs[], uint32_t num_bufs);
struct pkt_buf* pkt_buf_alloc(struct mempool* mempool);
void pkt_buf_free(struct pkt_buf* buf);

// reads the global VFIO container
int get_vfio_container();

// globally sets the VFIO container
void set_vfio_container(int fd);

}

#endif //IXY_MEMORY_H
