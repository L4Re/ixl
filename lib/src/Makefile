PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../..

DEFINES-$(CONFIG_IXL_NDEBUG) += -DIXL_NDEBUG

DEFINES += -D_DEFAULT_SOURCE

REQUIRES_LIBS = libpthread libio-vbus libc_support_misc libstdc++
# If chosen by the user, add jemalloc to the end of the libraries, catching all
# memory allocation calls.
REQUIRES_LIBS-$(CONFIG_IXL_JEMALLOC) += jemalloc

# Apparently, we need to define __STDC_LIMIT_MACROS to get all definitions from
# stdint.h, see also https://stackoverflow.com/questions/25561677/intmax-max-was-not-declared-in-this-scope?noredirect=1&lq=1
CXXFLAGS     += -D__STDC_LIMIT_MACROS

TARGET = libixl.a libixl.so
SRC_CC = log.cc \
         pci.cc \
         memory.cc \
         stats.cc \
         interrupts.cc \
         device.cc \
         driver/e1000/e1000.cc \
         driver/igb/igb.cc \
         driver/ixgbe/ixgbe.cc

# Include package-private header files
PRIVATE_INCDIR = $(PKGDIR)/lib/include

include $(L4DIR)/mk/lib.mk
