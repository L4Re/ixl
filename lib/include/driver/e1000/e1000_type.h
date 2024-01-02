/*****************************************************************************
 *                                                                           *
 *        e1000_type.h - Various definitions for driving an e1000 NIC        *
 *                                                                           *
 * Many of the contents are adapted from similar header files of DPDK and/or *
 * the Linux kernel. Moreover, the following document helped a lot with      *
 * figuring out several constants:                                           *
 * https://courses.cs.washington.edu/courses/cse451/16au/readings/e1000.pdf  *
 *                                                                           *
 * Copyright (C) 2023 Till Miemietz                                          *
 *                                                                           *
 *****************************************************************************/

#pragma once

#include <stdint.h>

/*
 * PCI device IDs
 */
#define E1000_DEV_ID_82540EM    0x100e

/*
 * Descriptor structures
 */

// Receive descriptor
struct e1000_rx_desc {
    uint64_t buf_addr;      /* Address of data buffer                         */
    uint16_t length;        /* Amount of data transferred into buffer via DMA */
    uint16_t csum;          /* Packet checksum                                */
    uint8_t  status;        /* Descriptor status                              */
    uint8_t  errors;        /* Descriptor errors                              */
    uint16_t spec;
};

// Transmit descriptor (adapted from Linux)
struct e1000_tx_desc {
    uint64_t buffer_addr;       /* Address of the descriptor's data buffer  */
    union {
        uint32_t data;
        struct {
            uint16_t length;    /* Data buffer length                       */
            uint8_t  cso;       /* Checksum offset                          */
            uint8_t  cmd;       /* Descriptor control                       */
        } flags;
    } lower;
    union {
        uint32_t data;
        struct {
            uint8_t  status;    /* Descriptor status                        */
            uint8_t  css;       /* Checksum start                           */
            uint16_t special;
        } fields;
    } upper;
};

// Receive descriptor bit definitions
#define E1000_RXD_STAT_DD   0x01    /* Descriptor done                      */
#define E1000_RXD_STAT_EOP  0x02    /* End of packet                        */

// Transmit descriptor bit definitions
#define E1000_TXD_STAT_DD   0x00000001  /* Descriptor done                  */
#define E1000_TXD_CMD_EOP   0x01000000  /* End of packet                    */
#define E1000_TXD_CMD_IFCS  0x02000000  /* Insert FCS (Ethernet checksum)   */
#define E1000_TXD_CMD_RS    0x08000000  /* Report status                    */

/*
 * E1000 device registers
 */

// General
#define E1000_CTRL      0x00000     /* Device control register              */
#define E1000_STATUS    0x00008     /* Device status register               */
#define E1000_EERD      0x00014     /* EEPROM read                          */

#define E1000_ICR       0x000C0     /* IRQ cause read                       */
#define E1000_ITR       0x000C4     /* IRQ throttling register              */
#define E1000_IMS       0x000D0     /* IRQ mask set register                */
#define E1000_IMC       0x000D8     /* IRQ mask clear                       */

#define E1000_RCTL      0x00100     /* RX control                           */
#define E1000_TCTL      0x00400     /* RX control                           */

#define E1000_RDBAL     0x02800     /* RX descr. base addr, low             */
#define E1000_RDBAH     0x02804     /* RX descr. base addr, high            */
#define E1000_RDLEN     0x02808     /* RX descr. length                     */
#define E1000_RDH       0x02810     /* RX descr. head                       */
#define E1000_RDT       0x02818     /* RX descr. tail                       */
#define E1000_RDTR      0x02820     /* RX delay timer register              */

#define E1000_TDBAL     0x03800     /* TX descr. base addr, low             */
#define E1000_TDBAH     0x03804     /* TX descr. base addr, high            */
#define E1000_TDLEN     0x03808     /* TX descr. length                     */
#define E1000_TDH       0x03810     /* TX descr. head                       */
#define E1000_TDT       0x03818     /* TX descr. tail                       */

// Statistics
#define E1000_GPRC      0x04074     /* Good packets received counter        */
#define E1000_GPTC      0x04080     /* Good packets transmitted counter     */
#define E1000_GORCL     0x04088     /* Good octets received count (low)     */
#define E1000_GORCH     0x0408C     /* Good octets received count (high)    */
#define E1000_GOTCL     0x04090     /* Good octets transmitted count (low)  */
#define E1000_GOTCH     0x04094     /* Good octets transmitted count (high) */

#define E1000_RXCSUM    0x05000     /* RX checksum (offload) control        */

#define E1000_RA        0x05400     /* Receive address array base addr      */

/*
 * E1000 device register content constants
 */

// E1000 CTRL register
#define E1000_CTRL_LRST     0x00000008  /* Link reset                       */
#define E1000_CTRL_ASDE     0x00000020  /* Auto-speed detect enable         */
#define E1000_CTRL_SLU      0x00000040  /* Set link up (force link)         */
#define E1000_CTRL_FRCSPD   0x00000800  /* Force speed                      */
#define E1000_CTRL_FRCDPX   0x00001000  /* Force duplex                     */
#define E1000_CTRL_RST      0x04000000  /* Global reset                     */

// E1000 STATUS register
#define E1000_STATUS_LU         0x00000002 /* Link up if flag is 1          */
#define E1000_STATUS_SPEED_MASK 0x000000C0
#define E1000_STATUS_SPEED_10   0x00000000 /* Link speed 10 Mbps            */
#define E1000_STATUS_SPEED_100  0x00000040 /* Link speed 100 Mbps           */
#define E1000_STATUS_SPEED_1000 0x00000080 /* Link speed 1000 Mbps          */

// E1000 interrupt conditions
#define E1000_ICR_RXDMT0      0x00000010   /* RX descriptor min threshold   */
#define E1000_ICR_RXO         0x00000040   /* Receiver overrun              */
#define E1000_ICR_RXT0        0x00000080   /* RX timer IRQ (ring 0)         */

// E1000 EERD register
#define E1000_EERD_START      0x00000001
#define E1000_EERD_DONE       0x00000010
#define E1000_EERD_ADDR_SHIFT 8
#define E1000_EERD_DATA_SHIFT 16

// E1000 receive control register
#define E1000_RCTL_EN       0x00000002  /* Enable RX path                   */
#define E1000_RCTL_UPE      0x00000008  /* Unicast promiscuous mode enable  */
#define E1000_RCTL_MPE      0x00000010  /* Multicast promiscuous mode enable*/
#define E1000_RCTL_BAM      0x00008000  /* Broadcast enable                 */

// E1000 transmit control register
#define E1000_TCTL_EN       0x00000002  /* Enable TX path                   */
#define E1000_TCTL_PSP      0x00000008  /* Pad short packets                */
#define E1000_TCTL_CT       0x00000ff0  /* Collision threshold bitmask      */
#define E1000_TCTL_COLD     0x003ff000  /* Collision distance bitmask       */

// RX checksum control register
#define E1000_RXCSUM_TUOFL  0x00000200  /* TCP & UDP csum offloading        */

// E1000 RA
#define E1000_RAH_AV        0x80000000  /* Receive descriptor is valid      */

// Collision parameter configuration
#define E1000_COLLISION_THRESHOLD 0xf
#define E1000_CT_SHIFT            4
#define E1000_COLLISION_DISTANCE  0x40  /* Relevant for half-duplex only    */
#define E1000_COLD_SHIFT          12
