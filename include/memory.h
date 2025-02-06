/*****************************************************************************
 *                                                                           *
 *    Memory - Utility functions for managing DMA-enabled (packet) memory.   *
 *                                                                           *
 * Some parts of this header still originate from the Ixy project, while the *
 * majority has been rewritten in C++ and was adapted to L4Re.               *
 *                                                                           *
 *****************************************************************************/

#pragma once

#include <atomic>
#include <mutex>

#include <l4/re/util/shared_cap>
#include <l4/re/dma_space>

#include <l4/ixylon/device.h>

namespace Ixl {

// Depend directly on the constants provided by L4. However, we define the size
// of IXL's huge pages in a central spot to facilitate adaptations later on.
const unsigned int HUGE_PAGE_BITS        = L4_SUPERPAGESHIFT;
const unsigned int HUGE_PAGE_SIZE        = L4_SUPERPAGESIZE;
const unsigned int SIZE_PKT_BUF_HEADROOM = (48 - sizeof(void *) - sizeof(std::atomic_uint32_t));

// Default packet buffer size in bytes
const unsigned int DFLT_PKTBUF_SIZE      = 2048;

/**
 * Container for returning the results of memory_allocate_dma().
 */
struct dma_memory {
    /// Cap to the newly created Dataspace
    L4Re::Util::Shared_cap<L4Re::Dataspace> mem_cap;

    /// Virtual addr at that the dataspace has been mapped in the calling task
    void* virt;
    /// Physical address of the dataspace as seen by the device
    L4Re::Dma_space::Dma_addr phy;
};

/**
 * Allocates a block of DMA-enabled memory and performs the necessary actions
 * for making this memory visible in the AS of the calling task as well as to
 * the target DMA device. Thus, the resulting memory block can be used for DMA
 * communication with the specified device.
 *
 * NOTE: When allocating memory, make sure that size obeys the DMA alignment
 *       constraints imposes by dev!
 *
 * \param dev  The DMA-capable device with that the memory shall be associated.
 * \param size Size of the memory allocation in bytes.
 *
 * \returns A container describing the allocated DMA-enabled memory.
 */
struct dma_memory memory_allocate_dma(Ixl_device& dev, size_t size);

/**
 * Structure that wraps a single packet handed over or received from the NIC.
 * As many NICs only support DMA to 64-Byte-aligned addresses, the start of the
 * data buffer has to be aligned to 64 B as well.
 */
struct pkt_buf {
    // physical address to pass a buffer to a nic
    L4Re::Dma_space::Dma_addr buf_addr_phy;

    // Memory pool that this packet buffer belongs to
    struct Mempool_chunk* mempool;
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

// Compile-time invariants needed for pkt_buf to work correctly with a NIC.
static_assert(sizeof(struct pkt_buf) == 64, "pkt_buf too large");
static_assert(offsetof(struct pkt_buf, data) == 64, "data at unexpected position");
static_assert(offsetof(struct pkt_buf, head_room) + SIZE_PKT_BUF_HEADROOM == offsetof(struct pkt_buf, data), "head room not immediately before data");

/*
 * A portion of a memory pool. Each chunk is in fact a
 * complete mempool implementation by itself and manages one contiguous
 * block of DMA-enabled memory allocated via memory_allocate_dma().
 */
struct Mempool_chunk {
    // Backing DMA memory (DMA-enabled Dataspace)
    struct dma_memory backing_mem;

    void* base_addr;

    uint32_t buf_size;
    uint32_t num_entries;

    // Memory is managed with a lock-free queue for indicating which buffers
    // are currently available. Note that this queue is only a list of array
    // indices pointing to each other (no real "data objects" are stored)
    // which allows for a very compressed representation. UINT32_MAX serves
    // as representation for an invalid index (i.e., queue end).
    std::atomic_uint32_t queue_head;
    std::atomic_uint32_t queue_tail;

    // The queue contains the entry id, i.e., base_addr + entry_id * buf_size
    // is the address of the buf
    std::atomic_uint32_t free_queue[];

    /**
     * Allocate several packet buffers from the mempool chunk at once.
     *
     * \param[out] bufs     Output array with starting addrs of the packet bufs.
     * \param[in]  num_bufs Number of buffers ot allocate.
     *
     * \returns The number of packets that could be allocated (<= num_bufs).
     */
    uint32_t pkt_buf_alloc_batch(struct pkt_buf* bufs[], uint32_t num_bufs);
};

/**
 * Return a packet buffer to the mempool chunk. Through pointers stored in
 * the packet buffer, the buffer will be returned to the Mempool_chunk from
 * that it was allocated. Note that any buffer passed to this function may be
 * immediately reused.
 *
 * \param buf The buffer to reclaim.
 */
void pkt_buf_free(struct pkt_buf* buf);

/**
 * A memory pool implementation for allocating packet buffers (i.e., data
 * structures of a fixes size). Internally, the memory pool consists of a list
 * of chunks that manage a separate DMA memory block each. The mempool allows
 * this list to grow dynamically, thus offering the opportunity to react to an
 * increasing number of clients that use the driver in parallel. This is
 * particularly important if said clients do not return packet buffers
 * immediately, as such situations may lead to OOM situations when using static
 * mempool sizes.
 *
 * TODO: Implement shrinking operations (might involve extensive algo rework).
 */
class Mempool {
public:
    /**
     * Create a new memory pool. The num entries and entry size paramters
     * define the shape of each mempool chunk ever allocated for this mempool.
     * Each chunk will accommodate num_entries elements with a size of
     * entry_size each. When calling the constructor, one such chunk will be
     * allocated. Subsequent allocation may be a consequence of increasing the
     * capacity of the mempool. The maximum number of chunks in this mempool
     * is given through the memory cap (mem_cap).
     *
     * \param dev         Device for whose DMA domain the memory is registered.
     * \param num_entries Number of entries per mempool_chunk.
     * \param entry_size  Size of each entry in the memory pool.
     * \param mem_cap     Limit of the overall capacity of the mempool in bytes.
     */
    Mempool(Ixl_device &dev, uint32_t num_entries, uint32_t entry_size,
            uint64_t mem_cap);

    /**
     * Allocate several packet buffers from the mempool at once.
     *
     * NOTE: This implementation may yield false negatives, i.e., when calling
     *       this function in parallel to the extension of the mempool, it may
     *       report OOM even though there is space in the mempool. In this case,
     *       calling the function again should resolve the error.
     *
     * \param[out] bufs     Output array with starting addrs of the packet bufs.
     * \param[in]  num_bufs Number of buffers ot allocate.
     *
     * \returns The number of packets that could be allocated (<= num_bufs).
     */
    uint32_t pkt_buf_alloc_batch(struct pkt_buf* bufs[], uint32_t num_bufs);

    /**
     * Allocate a single packet buffer from the mempool.
     *
     * NOTE: This implementation may yield false negatives, i.e., when calling
     *       this function in parallel to the extension of the mempool, it may
     *       report OOM even though there is space in the mempool. In this case,
     *       calling the function again should resolve the error.
     *
     * \returns The packet buffer address or NULL on error.
     */
    struct pkt_buf* pkt_buf_alloc(void);

    /**
     * Make sure that the mempool has sufficient capacity for returning
     * count more packet buffers.
     *
     * \param count Number of elements by that the mempool's size shall grow.
     *
     * \returns true on success, false else.
     */
    bool reserve(uint32_t count);

    /**
     * Tells the mempool that count elements of its capacity are no longer
     * needed (reverse operation of reserve). In the future, the mempool might
     * use this information to initiate a shrinking process.
     *
     * Note: Callers should make sure to only free reservations that were done
     *       beforehand (similar to malloc / free).
     *
     * \param count The number of elements by that the mempool's capacity may
     *              be decreased.
     */
    void cancel_reservation(uint32_t count);

private:
    // Device for that this mempool shall serve buffer requests
    Ixl_device &dev;

    /*
     * Array of mempool chunks. Through the memory cap of the pool, we can
     * figure out how many chunks there will be at most, saving some complexity
     * and/or overhead arising from shielding a vector against concurrent
     * access.
     */
    std::atomic_uint64_t *chunks;

    /*
     * Current chunk used for allocating memory. The mempool allocates from the
     * chunks using a revolving strategy. If one chunk is exhausted, subsequent
     * allocations will be serviced from the next one.
     */
    std::atomic_uint64_t cur_chunk;

    // Size of a single element managed by the memory pool.
    size_t elem_size;
    // Number of elements per pool chunk.
    size_t chunk_entries;
    // Limit of the overall memory size of this mempool in bytes.
    size_t mem_cap;
    // Maximum number of chunks possible in this mempool
    size_t max_chunks;

    // Mutex for protecting the variables storing the current usage level
    // of the mempool
    mutable std::mutex rsv_mtx;
    // Number of chunks currently used by the memory pool. Make this counter
    // atomic as allocation routine do not grab the rsv_mtx mutex.
    std::atomic_uint64_t chunk_cnt;
    // Number of elements currently reserved in the memory pool.
    size_t elems_reserved;

    /*
     * TODO: Introduce a hint to the next chunk that likely has free buffers
     *       available? This might be beneficial in situations where most of
     *       the mempool's elements are taken for (very) long periods. On the
     *       other hand, detecting an empty chunk is cheap in the current
     *       lock-free implementation...
     */

    /**
     * Allocate a new mempool chunk, together with the DMA-enabled memory
     * backing it. The dimensions of each chunk have been defined by the
     * parameters passed to the constructor of this class.
     *
     * \returns A populated mempool chunk.
     */
    struct Mempool_chunk* allocate_mempool_chunk(void);
};

} // namespace IXL
