/*****************************************************************************
 *                                                                           *
 *        igb_type.h - Various definitions for driving IGB-style NICs        *
 *                                                                           *
 * Many of the contents are adapted from similar header files of DPDK and/or *
 * the Linux kernel.                                                         *
 *                                                                           *
 * Copyright (C) 2023 Till Miemietz                                          *
 *                                                                           *
 *****************************************************************************/

#pragma once


/*
 * PCI device IDs
 */
#define IGB_DEV_ID_I350    0x1521

/*
 * Descriptor structures
 */

// Receive descriptor
struct igb_rx_desc {
    uint64_t buf_addr;      /* Address of data buffer                         */
    uint16_t length;        /* Amount of data transferred into buffer via DMA */
    uint16_t csum;          /* Packet checksum                                */
    uint8_t  status;        /* Descriptor status                              */
    uint8_t  errors;        /* Descriptor errors                              */
    uint16_t vlan_tag;
};

// Transmit descriptor (adapted from Linux)
struct igb_tx_desc {
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
#define IGB_RXD_STAT_DD   0x01      /* Descriptor done                      */
#define IGB_RXD_STAT_EOP  0x02      /* End of packet                        */

// Transmit descriptor bit definitions
#define IGB_TXD_STAT_DD   0x00000001    /* Descriptor done                  */
#define IGB_TXD_CMD_EOP   0x01000000    /* End of packet                    */
#define IGB_TXD_CMD_IFCS  0x02000000    /* Insert FCS (Ethernet checksum)   */
#define IGB_TXD_CMD_RS    0x08000000    /* Report status                    */

/*
 * Igb device registers
 */

// General
#define IGB_CTRL      0x00000       /* Device control register              */
#define IGB_STATUS    0x00008       /* Device status register               */
#define IGB_EERD      0x00014       /* EEPROM read                          */

#define IGB_ICR       0x000C0       /* IRQ cause read                       */
#define IGB_ITR       0x000C4       /* IRQ throttling register              */
#define IGB_IMS       0x000D0       /* IRQ mask set register                */
#define IGB_IMC       0x000D8       /* IRQ mask clear                       */

#define IGB_RCTL      0x00100       /* RX control                           */
#define IGB_SRIOV     0x00168       /* SR-IOV status and control register   */
#define IGB_TCTL      0x00400       /* RX control                           */

#define IGB_GPIE      0x01514       /* General purpose IRQ enable           */
#define IGB_EIMS      0x01524       /* Extended IRQ mask set                */
#define IGB_EIMC      0x01528       /* Extended IRQ mask clear              */
#define IGB_EIAC      0x0152C       /* Extended IRQ auto clear              */
#define IGB_EIAM      0x01530       /* Extended IRQ auto mask enable        */
#define IGB_EICR      0x01580       /* Extended IRQ cause                   */
#define IGB_EITR      0x01680       /* Extended IRQ throttle rate (base)    */
#define IGB_IVAR0     0x01700       /* IRQ vector allocation register 0     */

#define IGB_RDBAL     0x02800       /* RX descr. base addr, low             */
#define IGB_RDBAH     0x02804       /* RX descr. base addr, high            */
#define IGB_RDLEN     0x02808       /* RX descr. length                     */
#define IGB_SRRCTL0   0x0280C       /* Split and repl. RX ctrl reg queue 0  */
#define IGB_RDH       0x02810       /* RX descr. head                       */
#define IGB_RDT       0x02818       /* RX descr. tail                       */
#define IGB_RDTR      0x02820       /* RX delay timer register              */
#define IGB_RXDCTL0   0x02828       /* RX descr. ctrl. for queue 0          */

#define IGB_TDBAL     0x03800       /* TX descr. base addr, low             */
#define IGB_TDBAH     0x03804       /* TX descr. base addr, high            */
#define IGB_TDLEN     0x03808       /* TX descr. length                     */
#define IGB_TDH       0x03810       /* TX descr. head                       */
#define IGB_TDT       0x03818       /* TX descr. tail                       */
#define IGB_TXDCTL0   0x03828       /* TX descr. ctrl. for queue 0          */

// Statistics
#define IGB_GPRC      0x04074       /* Good packets received counter        */
#define IGB_GPTC      0x04080       /* Good packets transmitted counter     */
#define IGB_GORCL     0x04088       /* Good octets received count (low)     */
#define IGB_GORCH     0x0408C       /* Good octets received count (high)    */
#define IGB_GOTCL     0x04090       /* Good octets transmitted count (low)  */
#define IGB_GOTCH     0x04094       /* Good octets transmitted count (high) */

#define IGB_RXCSUM    0x05000       /* RX checksum (offload) control        */

#define IGB_RA        0x05400       /* Receive address array base addr      */

/*
 * Igb device register content constants
 */

// Igb CTRL register
#define IGB_CTRL_SLU      0x00000040    /* Set link up                      */
#define IGB_CTRL_FRCSPD   0x00000800    /* Force speed                      */
#define IGB_CTRL_FRCDPX   0x00001000    /* Force duplex                     */
#define IGB_CTRL_PRT_RST  0x04000000    /* Port reset                       */
#define IGB_CTRL_DEV_RST  0x20000000    /* Device reset                     */
#define IGB_CTRL_PHY_RST  0x80000000    /* PHY reset                        */

// Igb STATUS register
#define IGB_STATUS_LU         0x00000002   /* Link up if flag is 1          */
#define IGB_STATUS_SPEED_MASK 0x000000C0
#define IGB_STATUS_SPEED_10   0x00000000   /* Link speed 10 Mbps            */
#define IGB_STATUS_SPEED_100  0x00000040   /* Link speed 100 Mbps           */
#define IGB_STATUS_SPEED_1000 0x00000080   /* Link speed 1000 Mbps          */

// Igb interrupt conditions
#define IGB_ICR_RXT0        0x00000080     /* RX timer IRQ (ring 0)         */

// Igb EERD register
#define IGB_EERD_START      0x00000001
#define IGB_EERD_DONE       0x00000002
#define IGB_EERD_ADDR_SHIFT 2
#define IGB_EERD_DATA_SHIFT 16

// Igb SR-IOV control register
#define IGB_SRIOV_EN      0x00000001    /* SR-IOV enable                    */

// Igb receive control register
#define IGB_RCTL_EN       0x00000002    /* Enable RX path                   */
#define IGB_RCTL_UPE      0x00000008    /* Unicast promiscuous mode enable  */
#define IGB_RCTL_MPE      0x00000010    /* Multicast promiscuous mode enable*/
#define IGB_RCTL_BAM      0x00008000    /* Broadcast enable                 */
#define IGB_RCTL_VFE      0x00040000    /* VLAN fitler enable               */

// Igb receive descriptor control register
#define IGB_RXDCTL_EN     0x02000000    /* Receive queue enable             */

// Igb split and replication receive control registers
#define IGB_SRRCTL_DREN   0x80000000    /* Drop enabled if no descr. avail. */

// Igb transmit control register
#define IGB_TCTL_EN       0x00000002    /* Enable TX path                   */
#define IGB_TCTL_PSP      0x00000008    /* Pad short packets                */
#define IGB_TCTL_CT       0x00000ff0    /* Collision threshold bitmask      */

// Igb GPIE register
#define IGB_GPIE_NSICR    0x00000001    /* Non-selective IRQ clear on read  */
#define IGB_GPIE_MMSIX    0x00000010    /* Multiple MSI-X enable            */
#define IGB_GPIE_EIAME    0x40000000    /* Autoclear mask bit im EIAM       */
#define IGB_GPIE_PBA      0x80000000    /* PBA support                      */

// Igb transmit descriptor control register
#define IGB_TXDCTL_EN     0x02000000    /* Transmit queue enable            */

// RX checksum control register
#define IGB_RXCSUM_TUOFL  0x00000200    /* TCP & UDP csum offloading        */
#define IGB_RXCSUM_CRCOFL 0x00000800    /* CRC32 offload enable             */

// Igb RA
#define IGB_RAH_AV        0x80000000    /* Receive descriptor is valid      */

// Collision parameter configuration
#define IGB_COLLISION_THRESHOLD 0xf
#define IGB_CT_SHIFT            4
