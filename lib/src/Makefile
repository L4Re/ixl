PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../..

REQUIRES_LIBS = libio-vbus libc_support_misc

TARGET = libixylon.a libixylon.so
SRC_CC = pci.cc \
		 memory.cc \
		 stats.cc \
		 interrupts.cc \
		 libixy-vfio.cc \
		 device.cc \
		 driver/e1000/e1000.cc \
		 driver/ixgbe/ixgbe.cc

# Include package-private header files
PRIVATE_INCDIR = $(PKGDIR)/lib/include

include $(L4DIR)/mk/lib.mk
