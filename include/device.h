/*****************************************************************************
 *                                                                           *
 *        Device - The core device object of the Ixylon (IXL) driver.        *
 *                                                                           *
 * Some parts of this header still originate from the Ixy project, while the *
 * majority has been rewritten in C++ and was adapted to L4Re.               *
 *                                                                           *
 *****************************************************************************/

#pragma once

#include <stdint.h>
#include <unistd.h>

#include <string>

#include <l4/re/util/shared_cap>
#include <l4/vbus/vbus_pci>

#include "log.h"
#include "interrupts.h"

#define MAX_QUEUES 64

namespace Ixl {

/**
 * Maximum size of an IXL mempool used as backing memory for both RX and TX
 * queues of the NICs.
 *
 * TODO: Make this a configurable parameter, either on driver initialization,
 *       menuconfig, or at least static per NIC.
 */
const unsigned int MEMPOOL_LIMIT = 1 << 28;

struct __attribute__((__packed__)) mac_address {
    uint8_t addr[6];
};

/**
 * container_of - cast a member of a structure out to the containing structure
 * Adapted from the Linux kernel.
 * This allows us to expose the same struct for all drivers to the user's
 * application and cast it to a driver-specific struct in the driver.
 * A simple cast would be sufficient if we always store it at the same offset.
 * This macro looks more complicated than it is, a good explanation can be
 * found at http://www.kroah.com/log/linux/container_of.html
 * @ptr:    the pointer to the member.
 * @type:   the type of the container struct this is embedded in.
 * @member: the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({\
    const typeof(((type*)0)->member)* __mptr = (ptr);\
    (type*)((char*)__mptr - offsetof(type, member));\
})

// advance index with wrap-around
// this line is the reason why we require a power of two for the queue size
#define wrap_ring(index, ring_size) (uint16_t) ((index + 1) & (ring_size - 1))

/**
 * Wrapper structure that contains the (initial) configuration parameters of
 * an Ixl device. We use a dedicated config structure to reduce the number
 * of arguments passed to the ixl_init function. The structure comes with
 * default parameters that an application may change before using the config
 * with ixl_init().
 */
struct Dev_cfg {
    /// Number of NIC receive queues to allocate
    uint16_t num_rx_queues = 1;
    /// Number of NIC send queues to allocate
    uint16_t num_tx_queues = 1;

    /**
     * Timeout when waiting for IRQs from the NIC in ms. If set to zero, the
     * driver will use polling instead of IRQ-based event notification. If set
     * a negative value, the driver will set no IRQ timeout at all (potentially
     * sleeping forever).
     */
    int irq_timeout_ms = 1000;
};

/**
 * General abstraction for a device managed by an Ixylon driver.
 */
class Ixl_device {
public:
    /***                           Functions                              ***/

    /**
     * Returns the name of the driver.
     */
    virtual std::string get_driver_name(void) = 0;

    /**
     * Returns the maximum frame size that the device can currently handle.
     */
    virtual inline uint32_t get_max_frame_size(void) = 0;

    virtual uint32_t rx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                              uint32_t num_bufs) = 0;

    virtual uint32_t tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                              uint32_t num_bufs) = 0;

    virtual void read_stats(struct device_stats *stats) = 0;

    virtual void set_promisc(bool enabled) = 0;

    virtual uint32_t get_link_speed(void) = 0;

    virtual struct mac_address get_mac_addr(void) = 0;

    virtual void set_mac_addr(struct mac_address mac) = 0;

    // calls tx_batch until all packets are queued with busy waiting
    void tx_batch_busy_wait(uint16_t queue_id, struct pkt_buf* bufs[],
                            uint32_t num_bufs) {
        uint32_t num_sent = 0;
        while ((num_sent += tx_batch(queue_id, bufs + num_sent,
                                     num_bufs - num_sent)) != num_bufs) {
            // busy wait
        }
    }

    L4Re::Util::Shared_cap<L4Re::Dma_space> get_dma_space(void) {
        return dma_cap;
    }

    /**
     * Returns the number of RX queues allocated by the driver.
     */
    uint16_t get_num_rx_queues(void) {
        return num_rx_queues;
    }

    /**
     * Returns the number of descriptor entries per RX queue. This value is
     * set once upon device initialization.
     */
    virtual uint32_t get_rx_queue_depth(void) = 0;

    /**
     * Returns the number of TX queues allocated by the driver.
     */
    uint16_t get_num_tx_queues(void) {
        return num_tx_queues;
    }

    /**
     * Returns the number of descriptor entries per TX queue. This value is
     * set once upon device initialization.
     */
    virtual uint32_t get_tx_queue_depth(void) = 0;

    /**
     * Extend the memory pool backing RX queue qid by count packets.
     *
     * \param qid   Index of the receive queue affected by this operation.
     * \param count Number of packets by that the mempool shall be extended.
     *
     * \return true on success, false else.
     */
    virtual bool extend_rxq_mempool(uint16_t qid, uint32_t count) = 0;

    /**
     * Shrink the memory pool backing RX queue qid by count packets. Be careful
     * to only shrink a mempool by the amount that it has been extended before.
     * Otherwise, the driver may break.
     *
     * NOTE: Currently, this only clears reservation in the mempool, no actual
     *       shrinking is implemented.
     *
     * \param qid   Index of the receive queue affected by this operation.
     * \param count Number of packets by that the mempool shall be shrinked.
     */
    virtual void shrink_rxq_mempool(uint16_t qid, uint32_t count) = 0;

    /**
     * Binds the receive interrupt for the specified receive queue to the
     * calling thread. Note that by default, all receive interrupts are bound
     * to the thread that creates the device object, so if receive operations
     * are to be executed in a different thread, the thread calling rx_batch
     * needs to bind to the queue's interrupt first.
     *
     * \param qid Index of the receive queue affected by this operation.
     */
    void rebind_recv_irq(uint16_t qid);

    /**
     * Creates a new driver instance for a PCI device found on the vbus
     * passed to this function. After executing this function, the underlying
     * device shall be in an operational state and ready to handle send and
     * receive requests.
     *
     * \param vbus     Virtual device bus that shall be searched for
     *                 suitable PCI Ethernet devices
     * \param dev_idx  Index of the device to open (in case multiple
     *                 devices are present on the vbus)
     * \param cfg      Initial device configuration, see Dev_cfg for details.
     */
    static Ixl_device* ixl_init(L4::Cap<L4vbus::Vbus> vbus,
                                uint32_t dev_idx, struct Dev_cfg &cfg);

protected:
    /*                             Functions                                */

    /**
     * Create a DMA space for this device. The DMA space is later on needed
     * when making host memory accessible to the I/O device.
     *
     * Throws an exception if the underlying PCI device does not have any
     * DMA resources.
     */
    void create_dma_space(void);

    /**
     * Retrieves the cap to the virtualized ICU assigned to the vbus that
     * pci_dev is located on. This function needs to be called prior to any
     * other IRQ-related setup routines.
     */
    void setup_icu_cap(void);

    /***                        Member variables                          ***/

    // Memory addresses at that the resources described by the respective BARs
    // of the underlying PCI device got mapped. The index into the array
    // corresponds to the number of the BAR (i.e., baddr[0] is the start
    // address of the memory mapping for BAR0. BARs that are not mapped yet
    // are represented by NULL (it is also possible that some BARs are absent).
    uint8_t* baddr[6];

    void*    rx_queues;
    void*    tx_queues;

    uint16_t num_rx_queues;
    uint16_t num_tx_queues;
    struct   interrupts interrupts;

    // Underlying vbus PCI device handed over by L4
    L4vbus::Pci_dev pci_dev;

    // DMA space created for this device (needed to make standard dataspaces
    // accessible for I/O devices)
    L4Re::Util::Shared_cap<L4Re::Dma_space> dma_cap;
};

}
