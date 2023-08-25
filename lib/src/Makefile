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
		 driver/ixgbe.cc

# Include package-private header files
PRIVATE_INCDIR = $(PKGDIR)/lib/include

include $(L4DIR)/mk/lib.mk
