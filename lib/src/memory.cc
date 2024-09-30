/*****************************************************************************
 *                                                                           *
 *    Memory - Utility functions for managing DMA-enabled (packet) memory.   *
 *                                                                           *
 * Copyright (C) 2023, 2024 Till Miemietz                                    *
 *                          <till.miemietz@barkhauseninstitut.org>           *
 *                                                                           *
 *****************************************************************************/

#include <cstdint>
#include <exception>

#include <l4/sys/cxx/ipc_types>
#include <l4/re/error_helper>
#include <l4/re/env>
#include <l4/re/mem_alloc>

#include <l4/ixylon/memory.h>
#include <l4/ixylon/device.h>
#include <l4/ixylon/log.h>

using namespace Ixl;

// allocate memory suitable for DMA access in huge pages
struct dma_memory Ixl::memory_allocate_dma(Ixl_device& dev, size_t size) {
    // final DMA memory struct
    struct dma_memory ret;

    ret.mem_cap = L4Re::chkcap(L4Re::Util::make_shared_cap<L4Re::Dataspace>(),
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

// Allocate a batch of packet buffers from this chunk
uint32_t Ixl::Mempool_chunk::pkt_buf_alloc_batch(struct pkt_buf* bufs[],
                                                 uint32_t num_bufs) {

    for (uint32_t i = 0; i < num_bufs; i++) {
        // Do a pop() operation from the lock-free queue
        uint32_t entry_id = queue_head.load();

        while (entry_id != UINT32_MAX &&
               ! queue_head.compare_exchange_weak(entry_id,
                                                  free_queue[entry_id])) {
            entry_id = queue_head.load();
        }

        // Bail out if free pkt_buf queue is empty
        if (entry_id == UINT32_MAX)
            break;

        bufs[i] = (struct pkt_buf*) (((uint8_t*) base_addr) + entry_id * buf_size);
    }
    return num_bufs;
}

void Ixl::pkt_buf_free(struct pkt_buf* buf) {
    struct   Mempool_chunk* chunk = buf->mempool;
    uint32_t invalid              = UINT32_MAX;  // Marks invalid "pointer"
    bool     list_empty           = true;        // Is chunk's free_queue empty?

    // What follows now is a standard append routine for a lock-free queue...
    chunk->free_queue[buf->mempool_idx].store(invalid);
    uint32_t old_tail = chunk->queue_tail.load();

    while (! chunk->free_queue[old_tail].compare_exchange_weak(invalid,
                                                               buf->mempool_idx)) {
        list_empty = false;
        invalid    = UINT32_MAX;
        old_tail   = chunk->queue_tail.load();
    }

    chunk->queue_tail.compare_exchange_weak(old_tail, buf->mempool_idx);

    // Try to update head if list might have been empty. There may be false
    // positives for entering this branch e.g. on a newly initialized list.
    // In such cases however, no changes to the queue head should be done...
    if (list_empty) {
        invalid = UINT32_MAX;
        chunk->queue_head.compare_exchange_weak(invalid, buf->mempool_idx);
    }
}

// Mempool constructor
Ixl::Mempool::Mempool(Ixl_device &dev, uint32_t num_entries,
                      uint32_t entry_size, uint64_t mem_cap) :
                      dev(dev), elem_size(entry_size),
                      chunk_entries(num_entries), mem_cap(mem_cap) {
    // Initial mempool chunk allocated
    struct Mempool_chunk *init_chunk;

    // chunk_entries must not be greater than UINT32_MAX - 1, so that we can
    // safely use UINT32_MAX as invalid marker in the chunks free queues
    if (chunk_entries == UINT32_MAX)
        ixl_error("num_entries must not exceed UINT32_MAX - 1 per chunk!");

    elem_size = elem_size ? elem_size : DFLT_PKTBUF_SIZE;
    // require entries that neatly fit into the page size, this makes the
    // memory pool much easier. Otherwise our base_addr + index * size formula
    // would be wrong because we can't cross a page-boundary
    if (HUGE_PAGE_SIZE % elem_size) {
        ixl_error("entry size must be a divisor of the huge page size (%d)",
                  HUGE_PAGE_SIZE);
    }

    // Initially, there are no reservations
    elems_reserved = 0;

    // Allocate chunk array
    max_chunks = mem_cap / elem_size;
    chunks     = (std::atomic_uint64_t *) calloc(max_chunks,
                                                 sizeof(std::atomic_uint64_t));

    // Allocate the first chunk
    init_chunk = allocate_mempool_chunk();
    chunks[0].store((uint64_t) init_chunk);
    chunk_cnt  = 1;
    cur_chunk  = 0;
}

// Allocate a batch of packet buffers from a mempool.
uint32_t Ixl::Mempool::pkt_buf_alloc_batch(struct pkt_buf* bufs[],
                                           uint32_t num_bufs) {
    uint64_t idx;                              // Index of current chunk
    uint64_t tries          = chunk_cnt.load();// How many chunks to try?
    uint32_t pkts_allocated = 0;               // Packet buffer allocated so far

    for (unsigned int i = 0; i < tries; i++) {
        idx = cur_chunk.load();
        Mempool_chunk *chunk = (Mempool_chunk *) chunks[idx].load();

        // We should never reach this statement in a mempool that only grows
        // TODO: User assert instead to purge statement from release builds?
        if (chunk == NULL) {
            ixl_warn("Unexpected invalid chunk pointer in mempool!");
            break;
        }

        pkts_allocated += chunk->pkt_buf_alloc_batch(bufs + pkts_allocated,
                                                     num_bufs - pkts_allocated);

        if (pkts_allocated == num_bufs)
            break;

        // Try to advanced the chunk pointer by one (cmpxchg needed since
        // multiple threads may increment in parallel and we only want to
        // increment once)
        cur_chunk.compare_exchange_weak(idx, (idx + 1) % chunk_cnt);
    }

    return pkts_allocated;
}

// Allocate a single packet from a mempool.
struct pkt_buf* Ixl::Mempool::pkt_buf_alloc(void) {
    uint64_t        idx;                            // Index of current chunk
    uint64_t        tries = chunk_cnt.load();       // How many chunks to try?
    struct pkt_buf* buf   = NULL;

    // We try each chunk
    for (unsigned int i = 0; i < tries; i++) {
        idx = cur_chunk.load();
        Mempool_chunk *chunk = (Mempool_chunk *) chunks[idx].load();

        // We should never reach this statement in a mempool that only grows
        // TODO: User assert instead to purge statement from release builds?
        if (chunk == NULL) {
            ixl_warn("Unexpected invalid chunk pointer in mempool!");
            break;
        }

        chunk->pkt_buf_alloc_batch(&buf, 1);

        if (buf != NULL)
            break;

        // Try to advanced the chunk pointer by one (cmpxchg needed since
        // multiple threads may increment in parallel and we only want to
        // increment once)
        cur_chunk.compare_exchange_weak(idx, (idx + 1) % chunk_cnt);
    }

    return buf;
}

// Make sure that the mempool can service count more packet buffers.
bool Ixl::Mempool::reserve(uint32_t count) {
    uint32_t need_rsv = count;      // No. of elements waiting for a reservation

    std::lock_guard<std::mutex> lk(rsv_mtx);

    // Check if reservation would exceed mempool size limit
    if ((elems_reserved + count) > (max_chunks * chunk_entries))
        return false;

    // Is there already enough unreserved space in the mempool?
    if ((chunk_cnt * chunk_entries - elems_reserved) >= count) {
        elems_reserved += count;
        return true;
    }

    // We need to allocate more chunks...
    need_rsv -= chunk_cnt * chunk_entries - elems_reserved;
    while (true) {
        try {
            Mempool_chunk *chunk = allocate_mempool_chunk();

            chunks[chunk_cnt].store((uint64_t) chunk);
            chunk_cnt++;
        }
        catch (std::exception &e) {
            ixl_warn("Failed to allocate new memory chunk for mempool!");
            return false;
        }

        if (need_rsv > chunk_entries)
            need_rsv -= chunk_entries;
        else
            break;
    }

    elems_reserved += count;
    return true;
}

// Cancel a reservation in the mempool
void Ixl::Mempool::cancel_reservation(uint32_t count) {
    std::lock_guard<std::mutex> lk(rsv_mtx);

    // This would be the place to trigger a shrinking of the mempool.

    elems_reserved -= count;
}

// allocate a memory pool from which DMA'able packet buffers can be allocated
// entry_size can be 0 to use the default
struct Ixl::Mempool_chunk* Ixl::Mempool::allocate_mempool_chunk(void) {
    struct Mempool_chunk *chunk  =
            (struct Mempool_chunk *)
            calloc(1, sizeof(struct Mempool_chunk) +
                   chunk_entries * sizeof(std::atomic_uint32_t));

    chunk->backing_mem = memory_allocate_dma(dev, chunk_entries * elem_size);
    chunk->num_entries = chunk_entries;
    chunk->buf_size    = elem_size;
    chunk->base_addr   = chunk->backing_mem.virt;
    chunk->queue_head  = 0;
    chunk->queue_tail  = chunk_entries - 1;

    for (uint32_t i = 0; i < chunk_entries; i++) {
        struct pkt_buf* buf = (struct pkt_buf*) (((uint8_t*) chunk->base_addr) + i * elem_size);

        // Since the memory inside the DMA window was allocated physically
        // contiguously, we can just use an offset from the mempool's physical
        // base address to compute the physical address of each packet buffer
        buf->buf_addr_phy = chunk->backing_mem.phy + i * elem_size;

        buf->mempool_idx  = i;
        buf->mempool      = chunk;
        buf->size         = 0;

        // Chain all buffers in the list, letting the last one point to the
        // invalid array index
        // Since we have not handed out the mempool so far, it is safe to
        // set the array elements directly, without any MT atomic procedures
        if (i != (chunk_entries - 1))
            chunk->free_queue[i]  = i + 1;
        else
            chunk->free_queue[i]  = UINT32_MAX;
    }

    return chunk;
}
