PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../..

REQUIRES_LIBS = libio-vbus libc_support_misc

# Apparently, we need to define __STDC_LIMIT_MACROS to get all definitions from
# stdint.h, see also https://stackoverflow.com/questions/25561677/intmax-max-was-not-declared-in-this-scope?noredirect=1&lq=1
CXXFLAGS     += -D__STDC_LIMIT_MACROS

TARGET = libixylon.a libixylon.so
SRC_CC = log.cc \
         pci.cc \
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
