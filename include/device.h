#ifndef IXY_DEVICE_H
#define IXY_DEVICE_H

#include <stdint.h>
#include <unistd.h>

#include <string>

#include "log.h"
#include "memory.h"
#include "interrupts.h"

#define MAX_QUEUES 64

namespace Ixl {

struct __attribute__((__packed__)) mac_address {
	uint8_t	addr[6];
};

/**
 * container_of - cast a member of a structure out to the containing structure
 * Adapted from the Linux kernel.
 * This allows us to expose the same struct for all drivers to the user's
 * application and cast it to a driver-specific struct in the driver.
 * A simple cast would be sufficient if we always store it at the same offset.
 * This macro looks more complicated than it is, a good explanation can be
 * found at http://www.kroah.com/log/linux/container_of.html
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({\
	const typeof(((type*)0)->member)* __mptr = (ptr);\
	(type*)((char*)__mptr - offsetof(type, member));\
})

/**
 * General abstraction for a device managed by an Ixylon driver.
 */
class Ixl_device {
public:
    virtual std::string get_driver_name(void) = 0;

    virtual uint32_t rx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                              uint32_t num_bufs) = 0;

    virtual uint32_t tx_batch(uint16_t queue_id, struct pkt_buf* bufs[],
                              uint32_t num_bufs) = 0;

    virtual void read_stats(struct device_stats *stats) = 0;

    virtual void set_promisc(bool enabled) = 0;

    virtual uint32_t get_link_speed(void) = 0;

    virtual struct mac_address get_mac_addr(void) = 0;

    virtual void set_mac_addr(struct mac_address mac) = 0;

    // calls ixy_tx_batch until all packets are queued with busy waiting
    void tx_batch_busy_wait(uint16_t queue_id, struct pkt_buf* bufs[],
                            uint32_t num_bufs) {
        uint32_t num_sent = 0;
        while ((num_sent += tx_batch(queue_id, bufs + num_sent,
                                     num_bufs - num_sent)) != num_bufs) {
            // busy wait
        }
    }

    const char* get_pci_addr(void) {
        return pci_addr;
    }

    static Ixl_device* ixl_init(const char* pci_addr, uint16_t rx_queues,
                                uint16_t tx_queues, int irq_timeout);

protected:
    // TODO purge vfio stuff
	const char* pci_addr;
	uint16_t    num_rx_queues;
	uint16_t    num_tx_queues;
	bool        vfio;
	int         vfio_fd; // device fd
	struct      interrupts interrupts;
};

// getters/setters for PCIe memory mapped registers
// this code looks like it's in need of some memory barrier intrinsics, but that's apparently not needed on x86
// dpdk has release/acquire memory order calls before/after the memory accesses, but they are defined as
// simple compiler barriers (i.e., the same empty asm with dependency on memory as here) on x86
// dpdk also defines an additional relaxed load/store for the registers that only uses a volatile access,  we skip that for simplicity

static inline void set_reg32(uint8_t* addr, int reg, uint32_t value) {
	__asm__ volatile ("" : : : "memory");
	*((volatile uint32_t*) (addr + reg)) = value;
}

static inline uint32_t get_reg32(const uint8_t* addr, int reg) {
	__asm__ volatile ("" : : : "memory");
	return *((volatile uint32_t*) (addr + reg));
}

static inline void set_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_reg32(addr, reg, get_reg32(addr, reg) | flags);
}

static inline void clear_flags32(uint8_t* addr, int reg, uint32_t flags) {
	set_reg32(addr, reg, get_reg32(addr, reg) & ~flags);
}

static inline void wait_clear_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != 0) {
		ixl_debug("waiting for flags 0x%08X in register 0x%05X to clear, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

static inline void wait_set_reg32(const uint8_t* addr, int reg, uint32_t mask) {
	__asm__ volatile ("" : : : "memory");
	uint32_t cur = 0;
	while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != mask) {
		ixl_debug("waiting for flags 0x%08X in register 0x%05X, current value 0x%08X", mask, reg, cur);
		usleep(10000);
		__asm__ volatile ("" : : : "memory");
	}
}

// getters/setters for pci io port resources

static inline void write_io32(int fd, uint32_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		ixl_error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io16(int fd, uint16_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		ixl_error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline void write_io8(int fd, uint8_t value, size_t offset) {
	if (pwrite(fd, &value, sizeof(value), offset) != sizeof(value))
		ixl_error("pwrite io resource");
	__asm__ volatile("" : : : "memory");
}

static inline uint32_t read_io32(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint32_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		ixl_error("pread io resource");
	return temp;
}

static inline uint16_t read_io16(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint16_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		ixl_error("pread io resource");
	return temp;
}

static inline uint8_t read_io8(int fd, size_t offset) {
	__asm__ volatile("" : : : "memory");
	uint8_t temp;
	if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
		ixl_error("pread io resource");
	return temp;
}

}

#endif // IXY_DEVICE_H
