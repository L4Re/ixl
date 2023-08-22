PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../..

REQUIRES_LIBS = libc_support_misc

TARGET = libixylon.a libixylon.so
SRC_C  = pci.c \
		 memory.c \
		 stats.c \
		 interrupts.c \
		 libixy-vfio.c \
		 driver/device.c \
		 driver/ixgbe.c \
		 driver/virtio.c

# Include package-private header files
PRIVATE_INCDIR = $(PKGDIR)/lib/include

include $(L4DIR)/mk/lib.mk
