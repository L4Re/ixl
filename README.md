# Ixylon - A Collection of Userspace Network Drivers for L4Re

This package provides drivers for some types of network cards as well as
auxiliary functionality for driving such devices (DMA memory management,
interrupt setup, etc.). This package currently aims at bringing basic native
networking support to L4Re, meaning that you should be able to send
and receive packets. Thus, the driver implementation is currently missing
support for advanced features like offloading capabilities or SR-IOV.

Please also note that this implementation is geared towards performance on
larger machines. Thus some design decisions might not be suitable for small
systems with tight memory constraints.

## Supported Devices

See the table below for a list of device types supported by Ixylon:

| Driver | Supported NIC Types    | Remarks                               |
|--------|------------------------|---------------------------------------|
| e1000  | e1000-compliant NICs   | -                                     |
| igb    | Intel I350 Intel 82576 | Currently lacks multi-queue support.  |
| ixgbe  | Intel X520/X540        | -                                     |

## Using the Driver

The driver library uses three main abstractions. An `Ixl_device` represents
a network device run by the Ixylon driver. `Mempool`s represent DMA-enabled
memory that can be used together with an Ixylon device. Lastly, a `pkt_buf`
packet buffer structure encapsulates a single network packet sent or received
via an Ixylon device, along with some metadata handled by the driver.

### General Scheme of Operation

Binding the Ixylon driver to a device is done by calling the function
`ixl_init`. As first parameter, this function takes a capability to a virtual
bus (refer to the L4Re documentation for details). Upon success, `ixl_init`
returns an `Ixl_device` object, i.e., a handle to a fully initialized network
device. During initialization, the driver library automatically chooses the
correct driver to use with the device identified on the virtual bus, returning
an error if none was found. The initialization function furthermore expects
a `Dev_cfg` object that describes the extended configuration of the device.
Currently, the extended config allows for setting the number of receive and
send queues to allocate with the device, as well as the interrupt mode that
the driver should use (see below).

> Note: If multiple devices are present on the virtual bus passed to `ixl_init`,
> the driver always binds to the first device found.

As part of the initialization process, the driver resets and reconfigures the
network device. There currently is no dedicated function for graceful device
shutdown or driver unbinding at runtime. Rebinding a driver can be done by
killing the driver process bound to the network device and starting a new
driver.

Following successful setup, an `Ixl_device` provides the following operations:

* `get_max_frame_size`: Returns the maximum frame size supported by the network
                        device in bytes.
* `rx_batch`: Receives a number of network packets from a specific receive
              queue of the network device.
* `tx_batch`: Sends a number of network packets using a specific send
              queue of the network device.
* `read_stats`: Acquires basic statistics on device operation (currently the
                number of packets / bytes sent and received. Calling this
                function with a NULL pointer resets the device statistics.
                Otherwise, the counters are increased by the number of packets /
                bytes sent and received since the last time this function was
                called.
* `set_promisc`: Enables or disabled the promiscuous mode of the network device.
* `get_link_speed`: Returns the link speed in MiB/s.
* `get_mac_addr`: Returns the MAC address currently configured on the device.
* `set_mac_addr`: Sets the MAC address used by the device.
* `get_num_rx_queues`: Returns the number of receive queues running on the
                       device.
* `get_num_tx_queues`: Returns the number of send queues running on the
                       device.
* `get_rx_queue_depth`: Returns the number of descriptor entries per receive
                        queue. That is, the maximum number of outstanding
                        receive requests the the device can handle at once.
* `get_tx_queue_depth`: Returns the number of descriptor entries per send
                        queue. That is, the maximum number of outstanding
                        send requests the the device can handle at once.
* `extend_rxq_mempool`: Increases the capacity of the memory pool associated
                        with a specific receive queue of the device. See
                        below for details.
* `shrink_rxq_mempool`: Decreases the capacity of the memory pool associated
                        with a specific receive queue of the device. See below
                        for details.
* `get_recv_irq`: Returns a capability to the interrupt object of a specific
                  receive queue.
* `check_recv_irq`: Checks, clears, and masks the receive interrupt of a
                    specific receive queue.
* `ack_recv_irq`: Unmasks (i.e., re-enables) the receive interrupt of a
                  specific receive queue.
* `rebind_recv_irq`: Rebinds the receive interrupt of a specific receive queue
                     to the calling thread.

Please also refer to the testing applications in the `server` directory for
some examples on how to use the Ixylon driver in practice.

### Memory Management

Ixylon's memory pools are containers for a number of fixed-size memory objects.
Memory pools are always associated with an `Ixl_device` that is passed to its
constructor. During memory pool creation, the user can also specify the initial
number of entries and their size. Specifying an entry size of zero instructs
the memory pool to choose an entry size large enough to keep an Ixylon packet
buffer. After creation, it is possible to extend and shrink memory pools.

Every packet buffer used with an Ixylon device has to be allocated from
a Ixylon `Mempool` (as only this memory can be used for DMA transfers from and
to the network device). When a packet buffer is no longer needed, passing it
to the function `pkt_buf_free` returns it to the memory pool that it was
allocated from. Just as with `malloc` and `free`, leaking packet buffers will
exhaust the memory pools of a device and subsequently cause send and receive
operations to fail.

In general, all Ixylon device drivers allocate one memory pool for each receive
queue of a device. This pool serves for allocating packet buffers used for
receive requests, which are returned to the user as a result of the `rx_batch`
function. In order to allocate packet buffers for send operations, the user
has to create separate memory pools, which have to be associated with the
`Ixl_device` via that the respective packets shall be send. For optimal
performance, it is recommended to create one send memory pool per send queue of
the device.

### Interrupt Handling

Currently, the Ixylon driver provides interrupts that allow a thread to block
while waiting for incoming packets. For each receive queue of a device, a
separate receive interrupt can be allocated. During device initialization, the
user can set an interrupt mode for the device by means of the `irq_timeout_ms`
parameter of the device configuration struct. The interrupt mode applies to the
device as a whole.

If the IRQ timeout is set to zero, the driver will not use interrupts at all
and instead resort to polling in order to receive notifications about new
incoming packets from the network device. If the timeout is set to `-1`, the
driver allocates IRQ objects, which can be waited upon in a context external
to the driver (the IRQ object associated with a receive queue can be obtained
using the `get_recv_irq` function). Note that with this setting, the receive
function of the driver will not attempt to wait for notification via IRQs,
essentially implementing the same behavior as with polling.

If the IRQ timeout is set to any other negative value the driver enables
interrupts and uses an infinite timeout when waiting for incoming packets in
the receive function. This means that a thread calling `rx_batch` may be blocked
forever if the network device does not receive any data. If this infinite
wait time is not desired, a positive value of the IRQ timeout configures the
`rx_batch` function to abort the receive operation if no IRQ notification
was received after having waited for the IRQ timeout value (in ms).

> Note: Regardless of interrupt timeout settings, the Ixylon driver implements
> an adaptive use of interrupts. If the driver detects that the number of IRQs
> generated by the network device exceeds a certain rate, the driver
> automatically switches to polling mode, reducing the performance overhead
> introduced by excessive IRQ rates. Once the IRQ generation rate drops below
> the polling threshold, the driver restores the IRQ timeout settings as
> specified by the user.

## Running Ixylon in QEMU

For QEMU, use the following

    -net nic,model=e1000

to attach a network device compatible with the 'e1000' driver to your VM.

## Future Work

The following features were already identified to be missing and could be
implemented in the future:

* Allow DMA mempools to actually shrink (that is return parts of their memory).
* Propagate packet status flags to packet data structure (allows upper layers
  of the network stack to obtain additional information beyond success / fail).
* Provide an interface for querying and configuring currently active hardware
  features (partially implemented).
* Implement checksum offloading (requires use of advanced descriptors for all
  device types).
* Proper implementation of RSS if supported by NIC.
* Testing of driver on big endian ISAs.
* Re-enable Virtio driver from original ixy repository.

If you have implemented any of the features from the list above or extended
Ixylon with other additions, feel free to open a pull request. See also
the contributing guidelines in this repository for further information about
coding style etc.

## Acknowledgment

Parts of the source code are taken from the Ixy project
(https://github.com/emmericp/ixy), credits belong to the original authors. In
detail, this applies to the general code structure, the code for
adaptive interrupt enabling, and most of the implementation of the `ixgbe`
driver. See also the LICENSE file in this directory.
