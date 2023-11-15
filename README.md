# Ixylon - A simple userspace network driver for L4Re

Parts of the source code are taken from the Ixy project
(https://github.com/emmericp/ixy), credits belong to the original authors.
See also the LICENSE file in this directory.

### Run the Driver

If you are driving a NIC that makes use of MSIs (so pretty much every device
beside those found in emulated environments), make sure to start the `io`
service with the `--transparent-msi` flag.

### TODOs

* Make code work on big endian ISAs.
* Re-enable Virtio driver.
