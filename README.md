# Ixylon - A collection of simple userspace network drivers for L4Re

This package provides drivers for some types of network cards as well as
auxiliary functionality for driving such devices (DMA memory management,
interrupt setup, etc.). This package currently aims at bringing basic native
networking support to L4Re, meaning that you should be able to send
and receive packets. The implementation of driver support for advanced features
like SR-IOV is currently not planned.

Please also note that this implementation is geared towards performance on
larger machines. Thus some design decisions might not be suitable for small
systems with tight memory constraints.

### Supported Devices

See the table below for a list of device types supported by Ixylon:

| Driver | Supported NIC Types  | Remarks                               |
|--------|--------------------------------------------------------------|
| e1000  | e1000-compliant NICs | -                                     |
| igb    | Intel I350           | Currently lacks multi-queue support.  |
| ixgbe  | Intel X520           | L4Re port of this driver is untested. |

### Running the Driver

If you are driving a NIC that makes use of MSIs (so pretty much every device
beside those found in emulated environments), make sure to start the `io`
service with the `--transparent-msi` flag.

See also the testing applications in the `server` directory for some examples
on how to use the Ixylon driver.

### TODOs

The following (general) things are needed to make the driver more
sophisticated:

* Make DMA mempools dynamically resizable (Needed by upper layers to increase
  buffer space with growing number of clients).
* Propagate packet status flags to packet data structure (allows upper layers
  of the network stack to obtain additional information beyond success / fail).
* Provide an interface for querying and configuring currently active hardware
  features.
* Implement checksum offloading (requires use of advanced descriptors for all
  device types).
* Proper implementation of RSS if supported by NIC.
* Make code work on big endian ISAs.
* Re-enable Virtio driver.

### Acknowledgment

Parts of the source code are taken (though heavily changed) from the Ixy project
(https://github.com/emmericp/ixy), credits belong to the original authors.
See also the LICENSE file in this directory.
