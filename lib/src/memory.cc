// #include <linux/limits.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

#include <l4/sys/cxx/ipc_types>
#include <l4/re/error_helper>
#include <l4/re/env>
#include <l4/re/mem_alloc>

#include <l4/ixylon/memory.h>
#include <l4/ixylon/device.h>
#include <l4/ixylon/log.h>

using namespace Ixl;

// TODO: Purge vfio stuff
// we want one VFIO Container for all NICs, so every NIC can read from every
// other NICs memory, especially the mempool. When not using the IOMMU / VFIO,
// this variable is unused.
volatile int VFIO_CONTAINER_FILE_DESCRIPTOR = -1;

// allocate memory suitable for DMA access in huge pages
struct dma_memory Ixl::memory_allocate_dma(Ixl_device& dev, size_t size) {
    // final DMA memory struct
    struct dma_memory ret;

    ret.mem_cap = L4Re::chkcap(L4Re::Util::make_unique_cap<L4Re::Dataspace>(),
                               "Failed to allocate cap for I/O memory.");

    ixl_debug("allocating dma memory via huge page");
    // round up to multiples of 2 MB if necessary, this is the wasteful part
    // this could be fixed by co-locating allocations on the same page until a request would be too large
    // when fixing this: make sure to align on 128 byte boundaries (82599 dma requirement)
    if (size % HUGE_PAGE_SIZE) {
        size = ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_BITS;
    }

    // Allocate and attach the backing dataspace to our AS. Note that L4 has
    // transparent hugepage support, so no further measures have to be taken
    // in this regard.
    // We require the memory to be continuous and pinned to make it suitable
    // for DMA operations
    L4Re::chksys(L4Re::Env::env()->mem_alloc()->alloc(size, ret.mem_cap.get(),
                        L4Re::Mem_alloc::Continuous | L4Re::Mem_alloc::Pinned),
                        "Failed to allocate DMA memory.");
    // Align to 2 MiB huge pages
    L4Re::chksys(L4Re::Env::env()->rm()->attach(&ret.virt, size,
                        L4Re::Rm::F::Search_addr | L4Re::Rm::F::RW,
                        L4::Ipc::make_cap_rw(ret.mem_cap.get()), 0,
                        L4_SUPERPAGESHIFT),
                        "Failed to attach DMA memory");

    // Finally, enable DMA accesses to the memory region
    // TODO: We should check the return value of size to see whether we have
    //       to call map again...
    L4Re::chksys(dev.get_dma_space()->map(
                        L4::Ipc::make_cap_rw(ret.mem_cap.get()), 0,
                        &size, L4Re::Dma_space::Attributes::None, 
                        L4Re::Dma_space::Direction::Bidirectional,
                        &ret.phy),
                        "Failed to setup memory region for DMA.");

    return ret;
}

// allocate a memory pool from which DMA'able packet buffers can be allocated
// this is currently not yet thread-safe, i.e., a pool can only be used by one thread,
// this means a packet can only be sent/received by a single thread
// entry_size can be 0 to use the default
struct mempool* Ixl::memory_allocate_mempool(Ixl_device& dev,
                                             uint32_t num_entries,
                                             uint32_t entry_size) {
    entry_size = entry_size ? entry_size : 2048;
    // require entries that neatly fit into the page size, this makes the memory pool much easier
    // otherwise our base_addr + index * size formula would be wrong because we can't cross a page-boundary
    if (HUGE_PAGE_SIZE % entry_size) {
        ixl_error("entry size must be a divisor of the huge page size (%d)",
                  HUGE_PAGE_SIZE);
    }
    struct mempool* mempool = (struct mempool*) malloc(sizeof(struct mempool) + num_entries * sizeof(uint32_t));
    mempool->backing_mem    = memory_allocate_dma(dev, num_entries * entry_size);
    mempool->num_entries    = num_entries;
    mempool->buf_size       = entry_size;
    mempool->base_addr      = mempool->backing_mem.virt;
    mempool->free_stack_top = num_entries;

    for (uint32_t i = 0; i < num_entries; i++) {
        mempool->free_stack[i] = i;
        struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) mempool->base_addr) + i * entry_size);
        
        // Since the memory inside the DMA window was allocated physically 
        // contiguously, we can just use an offset from the mempool's physical
        // base address to compute the physical address of each packet buffer
        buf->buf_addr_phy = mempool->backing_mem.phy + i * sizeof(entry_size);
        
        buf->mempool_idx  = i;
        buf->mempool      = mempool;
        buf->size         = 0;
    }

    return mempool;
}

uint32_t Ixl::pkt_buf_alloc_batch(struct mempool* mempool, struct pkt_buf* bufs[], uint32_t num_bufs) {
    if (mempool->free_stack_top < num_bufs) {
        ixl_warn("memory pool %p only has %d free bufs, requested %d", mempool, mempool->free_stack_top, num_bufs);
        num_bufs = mempool->free_stack_top;
    }
    for (uint32_t i = 0; i < num_bufs; i++) {
        uint32_t entry_id = mempool->free_stack[--mempool->free_stack_top];
        bufs[i] = (struct pkt_buf*) (((uint8_t*) mempool->base_addr) + entry_id * mempool->buf_size);
    }
    return num_bufs;
}

struct pkt_buf* Ixl::pkt_buf_alloc(struct mempool* mempool) {
    struct pkt_buf* buf = NULL;
    pkt_buf_alloc_batch(mempool, &buf, 1);
    return buf;
}

void Ixl::pkt_buf_free(struct pkt_buf* buf) {
    struct mempool* mempool = buf->mempool;
    mempool->free_stack[mempool->free_stack_top++] = buf->mempool_idx;
}

// reads the global VFIO container
int Ixl::get_vfio_container() {
    return VFIO_CONTAINER_FILE_DESCRIPTOR;
}

// globally sets the VFIO container and returns the set value
void Ixl::set_vfio_container(int fd) {
    VFIO_CONTAINER_FILE_DESCRIPTOR = fd;
}
