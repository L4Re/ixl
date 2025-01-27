PKGDIR ?= ../..
L4DIR  ?= $(PKGDIR)/../..

# Check whether release flag is set
ifeq ($(CONFIG_IXL_NDEBUG), y)
    DEFINES += -DIXL_NDEBUG
endif

# If chosen by the user, add jemalloc to the end of the libraries, catching all
# memory allocation calls.
REQUIRES_LIBS = libpthread libio-vbus libc_support_misc libstdc++
ifeq ($(CONFIG_IXL_JEMALLOC),y)
REQUIRES_LIBS += jemalloc
endif

# Apparently, we need to define __STDC_LIMIT_MACROS to get all definitions from
# stdint.h, see also https://stackoverflow.com/questions/25561677/intmax-max-was-not-declared-in-this-scope?noredirect=1&lq=1
CXXFLAGS     += -D__STDC_LIMIT_MACROS

TARGET = libixylon.a libixylon.so
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
