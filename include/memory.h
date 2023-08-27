#ifndef IXY_MEMORY_H
#define IXY_MEMORY_H

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <l4/re/util/unique_cap>
#include <l4/re/dma_space>

#include <l4/ixylon/device.h>

namespace Ixl {

#define HUGE_PAGE_BITS 21
#define HUGE_PAGE_SIZE (1 << HUGE_PAGE_BITS) // 2_097_152 = 2MiB
#define SIZE_PKT_BUF_HEADROOM 40

struct pkt_buf {
    // physical address to pass a buffer to a nic
    L4Re::Dma_space::Dma_addr buf_addr_phy;
    struct mempool* mempool;
    uint32_t mempool_idx;
    uint32_t size;
    uint8_t head_room[SIZE_PKT_BUF_HEADROOM];
    uint8_t data[] __attribute__((aligned(64)));
};

static_assert(sizeof(struct pkt_buf) == 64, "pkt_buf too large");
static_assert(offsetof(struct pkt_buf, data) == 64, "data at unexpected position");
static_assert(offsetof(struct pkt_buf, head_room) + SIZE_PKT_BUF_HEADROOM == offsetof(struct pkt_buf, data), "head room not immediately before data");

struct dma_memory {
    L4Re::Util::Unique_cap<L4Re::Dataspace> mem_cap;

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
    // memory is managed via a simple stack
    // replacing this with a lock-free queue (or stack) makes this thread-safe
    uint32_t free_stack_top;
    // the stack contains the entry id, i.e., base_addr + entry_id * buf_size is the address of the buf
    uint32_t free_stack[];
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
