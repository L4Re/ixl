/*****************************************************************************
 *                                                                           *
 *          igc_type.h - Various definitions for driving igc-style NICs      *
 *                                                                           *
 * Copyright (C) 2025 Richter Paul <paul.richter@spreewalddreieck.de>        *
 *                                                                           *
 * Many of the contents are adapted from similar header files of the Linux   *
 * kernel, ixl igb driver and the i225 datasheet.                            * 
 *                                                                           *
 * The igc device driver was developed as part of the bachelor thesis:       *
 * "Generic aspects of porting a Linux Ethernet driver to the                *
 * L4Re microkernel" It is nearly an identical copy of the ixl igb driver    *
 * with changed register definitions. More information about how it works    *
 * can be found there.                                                       *
 *                                                                           *
 * https://github.com/hockeyfriend/Generic-aspects-of-porting-a-Linux-Ethernet-driver-to-the-L4Re-microkernel *
 *
 *****************************************************************************/

#ifndef _IGC_TYPE_H_
#define _IGC_TYPE_H_

// Receive descriptor
struct igc_rx_desc {
    uint64_t buf_addr;      /* Address of data buffer                         */
    uint16_t length;        /* Amount of data transferred into buffer via DMA */
    uint16_t csum;          /* Packet checksum                                */
    uint8_t  status;        /* Descriptor status                              */
    uint8_t  errors;        /* Descriptor errors                              */
    uint16_t vlan_tag;
};

// Transmit descriptor (copied from igb driver)
struct igc_tx_desc {
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

#define IGC_DEV_ID_I225_LM			0x15F2
#define IGC_DEV_ID_I225_V			0x15F3
#define IGC_DEV_ID_I225_I			0x15F8
#define IGC_DEV_ID_I220_V			0x15F7
#define IGC_DEV_ID_I225_K			0x3100
#define IGC_DEV_ID_I225_K2			0x3101
#define IGC_DEV_ID_I226_K			0x3102
#define IGC_DEV_ID_I225_LMVP	    0x5502
#define IGC_DEV_ID_I226_LMVP		0x5503
#define IGC_DEV_ID_I225_IT			0x0D9F
#define IGC_DEV_ID_I226_LM			0x125B
#define IGC_DEV_ID_I226_V			0x125C
#define IGC_DEV_ID_I226_IT			0x125D
#define IGC_DEV_ID_I221_V			0x125E
#define IGC_DEV_ID_I226_BLANK_NVM	0x125F
#define IGC_DEV_ID_I225_BLANK_NVM	0x15FD

// General registers (taken from Linux IGC driver /igc_regs.h line 7)
#define IGC_CTRL            0x00000  /* Device Control - RW                   */
#define IGC_STATUS          0x00008  /* Device Status - RO                    */
#define IGC_EECD            0x00010  /* EEPROM/Flash Control - RW             */
#define IGC_CTRL_EXT        0x00018  /* Extended Device Control - RW          */
#define IGC_MDIC            0x00020  /* MDI Control - RW                      */
#define IGC_CONNSW          0x00034  /* Copper/Fiber switch control - RW      */
#define IGC_VET             0x00038  /* VLAN Ether Type - RW                  */
#define IGC_LEDCTL          0x00E00  /* LED Control - RW                      */
#define IGC_I225_PHPM       0x00E14  /* I225 PHY Power Management             */
#define IGC_GPHY_VERSION    0x0001E  /* I225 gPHY Firmware Version            */

// igc_regs.h line 44
/* Interrupt Register Description */
#define IGC_EICR		0x01580  /* Ext. Interrupt Cause read - W0            */
#define IGC_EICS		0x01520  /* Ext. Interrupt Cause Set - W0             */
#define IGC_EIMS		0x01524  /* Ext. Interrupt Mask Set/Read - RW         */
#define IGC_EIMC		0x01528  /* Ext. Interrupt Mask Clear - WO            */
#define IGC_EIAC		0x0152C  /* Ext. Interrupt Auto Clear - RW            */
#define IGC_EIAM		0x01530  /* Ext. Interrupt Auto Mask - RW             */
#define IGC_ICR			0x01500  /* Intr Cause Read - RC/W1C                  */
#define IGC_ICS			0x01504  /* Intr Cause Set - WO                       */
#define IGC_IMS			0x01508  /* Intr Mask Set/Read - RW                   */
#define IGC_IMC			0x0150C  /* Intr Mask Clear - WO                      */
#define IGC_IAM			0x01510  /* Intr Ack Auto Mask- RW                    */
/* Intr Throttle - RW */
#define IGC_EITR		0x01680
/* Interrupt Vector Allocation - RW */
#define IGC_IVAR0		0x01700
#define IGC_IVAR_MISC	0x01740  /* IVAR for "other" causes - RW              */
#define IGC_GPIE		0x01514  /* General Purpose Intr Enable - RW          */

// igc_defines.h 304
/* Receive Descriptor bit definitions */
#define IGC_RXD_STAT_DD		0x01  /* Descriptor Done */
// igc_defines.h 370
#define IGC_RXD_STAT_EOP	0x02  /* End of Packet */
//
#define IGC_RDH		0x0C010
#define IGC_RDT		0x0C018

// (igc_regs.h line 64)
/* RSS registers */
#define IGC_MRQC		0x05818  /* Multiple Receive Control - RW */

// (igc_defines.h line 386)
#define IGC_MRQC_ENABLE_RSS_MQ		0x00000002

/* transmit descriptor defines */
// (igc_regs.h line 114)
#define IGC_TCTL		0x00400  /* Tx Control - RW */
#define IGC_TDBAL       0x0E000
#define IGC_TDBAH       0x0E004
#define IGC_TDLEN       0x0E008
#define IGC_TDH         0x0E010
#define IGC_TDT         0x0E018
#define IGC_TXDCTL0     0x0E028

// adapted from igb driver
#define IGC_TXDCTL_EN   0x02000000

// adapated from igb RX checksum control register
#define IGC_RXCSUM_TUOFL  0x00000200    /* TCP & UDP csum offloading        */
#define IGC_RXCSUM_CRCOFL 0x00000800    /* CRC32 offload enable             */

// (igc_defines.h 313)
/* Transmit descriptor defines */
#define IGC_TXD_CMD_EOP		0x01000000 /* End of Packet */
#define IGC_TXD_STAT_DD		0x00000001 /* Descriptor Done */

// copied from igb driver
#define IGC_TXD_CMD_IFCS    0x02000000
#define IGC_TXD_CMD_RS      0x08000000

// (igc_defines.h 331)
/* Transmit Control */
#define IGC_TCTL_EN		0x00000002 /* enable Tx */
#define IGC_TCTL_PSP	0x00000008 /* pad short packets */
#define IGC_TCTL_CT		0x00000ff0 /* collision threshold */

// (igc_defines.h 217)
/* Collision related configuration parameters */
#define IGC_COLLISION_THRESHOLD		15
#define IGC_CT_SHIFT			4

// (igc_regs.h line 223).

// Device Status
#define IGC_STATUS_FD		         0x00000001  /* Full duplex.0=half,1=full */
#define IGC_STATUS_LU		         0x00000002  /* Link up.0=no,1=link       */
#define IGC_STATUS_FUNC_MASK	     0x0000000C  /* PCI Function Mask         */
#define IGC_STATUS_FUNC_SHIFT	     2
#define IGC_STATUS_TXOFF	         0x00000010  /* transmission paused       */
#define IGC_STATUS_SPEED_100	     0x00000040  /* Speed 100Mb/s             */
#define IGC_STATUS_SPEED_1000	     0x00000080  /* Speed 1000Mb/s            */
#define IGC_STATUS_SPEED_2500	     0x00400000	 /* Speed 2.5Gb/s             */

#define SPEED_10		10
#define SPEED_100		100
#define SPEED_1000		1000
#define SPEED_2500		2500
#define HALF_DUPLEX		1
#define FULL_DUPLEX		2

typedef enum
{
    LINK_SPEED_UNDEFINED = 0,
    LINK_SPEED_10 = 10      /* 10 MiB / s */,
    LINK_SPEED_100 = 100    /* 100 MiB / s */,
    LINK_SPEED_1000 = 1000  /* 1000 MiB / s */,
    LINK_SPEED_2500 = 2500 /* 2500 MiB / s*/,
} EnumLinkSpeed;

typedef enum{
    LINK_DUPLEX_UNDEFINED = 0,
    LINK_DUPLEX_HALF = 1,
    LINK_DUPLEX_FULL = 2
} EnumLinkDuplex;

// copied from igb
#define IGC_CTRL_DEV_RST    0x20000000

// copied from igb 
/* general purpose interrupt enable */
#define IGC_GPIE_NSICR      0x00000001
#define IGC_GPIE_MMSIX      0x00000010
#define IGC_GPIE_EIAME      0x40000000
#define IGC_GPIE_PBA        0x80000000

// taken from igc_defines.h 131
/* Device Control */  
#define IGC_CTRL_RST		0x04000000  /* Global reset */

#define IGC_CTRL_PHY_RST	0x80000000  /* PHY Reset */
#define IGC_CTRL_SLU		0x00000040  /* Set link up (Force Link) */
#define IGC_CTRL_FRCSPD		0x00000800  /* Force Speed */
#define IGC_CTRL_FRCDPX		0x00001000  /* Force Duplex */

// Receive registers (taken from igc_defines.h line 347 and igc_regs.h line 98).
#define IGC_RCTL		                0x00100  /* Rx Control - RW */
#define IGC_RCTL_RST		            0x00000001 /* Software reset */
#define IGC_RCTL_EN		                0x00000002 /* enable */
#define IGC_RCTL_SBP		            0x00000004 /* store bad packet */
#define IGC_RCTL_UPE		            0x00000008 /* unicast promisc enable */
#define IGC_RCTL_MPE		            0x00000010 /* multicast promisc enable */
#define IGC_RCTL_LPE		            0x00000020 /* long packet enable */
#define IGC_RCTL_LBM_MAC	            0x00000040 /* MAC loopback mode */
#define IGC_RCTL_LBM_TCVR	            0x000000C0 /* tcvr loopback mode */
#define IGC_RCTL_RDMTS_HALF	            0x00000000 /* Rx desc min thresh size */
#define IGC_RCTL_BAM		            0x00008000 /* broadcast enable */

// from igb linux header confirmed with i225 datasheet that bit mask is the same
#define IGC_RCTL_VFE                    0x00040000 /* VLAN Filter */                      

#define IGC_SRRCTL0	                    0x0C00C // adapted from igc_regs.h line 94
#define IGC_PSRTYPE 		            0x05480
#define IGC_RDBAL   		            0x0C000
#define IGC_RDBAH   		            0x0C004
#define IGC_RDLEN   		            0x0C008
#define IGC_RXDCTL0  		            0x0C028
#define IGC_RQDPC   		            0x0C030
#define IGC_RXCSUM		                0x05000  /* Rx Checksum Control - RW */
#define IGC_RLPML		                0x05004  /* Rx Long Packet Max Length */
#define IGC_RFCTL		                0x05008  /* Receive Filter Control*/
#define IGC_MTA			                0x05200  /* Multicast Table Array - RW Array */
#define IGC_RA			                0x05400  /* Receive Address - RW Array */
#define IGC_UTA			                0x0A000  /* Unicast Table Array - RW */
#define IGC_RAL(_n)		                (0x05400 + ((_n) * 0x08))
#define IGC_RAL_MAC_ADDR_LEN	        4
#define IGC_RAH(_n)		                (0x05404 + ((_n) * 0x08))
#define IGC_RAH_MAC_ADDR_LEN	        2
#define IGC_VLANPQF		                0x055B0  /* VLAN Priority Queue Filter - RW */

#define IGC_RXDCTL_EN		0x02000000 // adapted from igc.h line 491

// copied from igb crossreferenced in datasheet to ensure that it's the same for
// igc
#define IGC_SRRCTL_DREN     0x80000000    /* Drop enabled if no descr. avail. */

#define ETH_ALEN                        6 /* length of Ethernet/MAC address */

// Statistics registers.
#define IGC_CRCERRS	                    0x04000  /* CRC Error Count - R/clr */
#define IGC_ALGNERRC	                0x04004  /* Alignment Error Count - R/clr */
#define IGC_RXERRC	                    0x0400C  /* Receive Error Count - R/clr */
#define IGC_MPC							0x04010  /* Missed Packet Count - R/clr */
#define IGC_SCC							0x04014  /* Single Collision Count - R/clr */
#define IGC_ECOL						0x04018  /* Excessive Collision Count - R/clr */
#define IGC_MCC							0x0401C  /* Multiple Collision Count - R/clr */
#define IGC_LATECOL						0x04020  /* Late Collision Count - R/clr */
#define IGC_COLC						0x04028  /* Collision Count - R/clr */
#define IGC_RERC						0x0402C  /* Receive Error Count - R/clr */
#define IGC_DC							0x04030  /* Defer Count - R/clr */
#define IGC_TNCRS						0x04034  /* Tx-No CRS - R/clr */
#define IGC_HTDPMC						0x0403C  /* Host Transmit Discarded by MAC - R/clr */
#define IGC_RLEC						0x04040  /* Receive Length Error Count - R/clr */
#define IGC_XONRXC						0x04048  /* XON Rx Count - R/clr */
#define IGC_XONTXC						0x0404C  /* XON Tx Count - R/clr */
#define IGC_XOFFRXC						0x04050  /* XOFF Rx Count - R/clr */
#define IGC_XOFFTXC						0x04054  /* XOFF Tx Count - R/clr */
#define IGC_FCRUC						0x04058  /* Flow Control Rx Unsupported Count- R/clr */
#define IGC_PRC64						0x0405C  /* Packets Rx (64 bytes) - R/clr */
#define IGC_PRC127						0x04060  /* Packets Rx (65-127 bytes) - R/clr */
#define IGC_PRC255						0x04064  /* Packets Rx (128-255 bytes) - R/clr */
#define IGC_PRC511						0x04068  /* Packets Rx (255-511 bytes) - R/clr */
#define IGC_PRC1023						0x0406C  /* Packets Rx (512-1023 bytes) - R/clr */
#define IGC_PRC1522						0x04070  /* Packets Rx (1024-1522 bytes) - R/clr */
#define IGC_GPRC						0x04074  /* Good Packets Rx Count - R/clr */
#define IGC_BPRC						0x04078  /* Broadcast Packets Rx Count - R/clr */
#define IGC_MPRC						0x0407C  /* Multicast Packets Rx Count - R/clr */
#define IGC_GPTC						0x04080  /* Good Packets Tx Count - R/clr */
#define IGC_GORCL						0x04088  /* Good Octets Rx Count Low - R/clr */
#define IGC_GORCH						0x0408C  /* Good Octets Rx Count High - R/clr */
#define IGC_GOTCL						0x04090  /* Good Octets Tx Count Low - R/clr */
#define IGC_GOTCH						0x04094  /* Good Octets Tx Count High - R/clr */
#define IGC_RNBC						0x040A0  /* Rx No Buffers Count - R/clr */
#define IGC_RUC							0x040A4  /* Rx Undersize Count - R/clr */
#define IGC_RFC							0x040A8  /* Rx Fragment Count - R/clr */
#define IGC_ROC							0x040AC  /* Rx Oversize Count - R/clr */
#define IGC_RJC							0x040B0  /* Rx Jabber Count - R/clr */
#define IGC_MGTPRC						0x040B4  /* Management Packets Rx Count - R/clr */
#define IGC_MGTPDC						0x040B8  /* Management Packets Dropped Count - R/clr */
#define IGC_MGTPTC						0x040BC  /* Management Packets Tx Count - R/clr */
#define IGC_TORL						0x040C0  /* Total Octets Rx Low - R/clr */
#define IGC_TORH						0x040C4  /* Total Octets Rx High - R/clr */
#define IGC_TOTL						0x040C8  /* Total Octets Tx Low - R/clr */
#define IGC_TOTH						0x040CC  /* Total Octets Tx High - R/clr */
#define IGC_TPR							0x040D0  /* Total Packets Rx - R/clr */
#define IGC_TPT							0x040D4  /* Total Packets Tx - R/clr */
#define IGC_PTC64						0x040D8  /* Packets Tx (64 bytes) - R/clr */
#define IGC_PTC127						0x040DC  /* Packets Tx (65-127 bytes) - R/clr */
#define IGC_PTC255						0x040E0  /* Packets Tx (128-255 bytes) - R/clr */
#define IGC_PTC511						0x040E4  /* Packets Tx (256-511 bytes) - R/clr */
#define IGC_PTC1023						0x040E8  /* Packets Tx (512-1023 bytes) - R/clr */
#define IGC_PTC1522						0x040EC  /* Packets Tx (1024-1522 Bytes) - R/clr */
#define IGC_MPTC						0x040F0  /* Multicast Packets Tx Count - R/clr */
#define IGC_BPTC						0x040F4  /* Broadcast Packets Tx Count - R/clr */
#define IGC_TSCTC						0x040F8  /* TCP Segmentation Context Tx - R/clr */
#define IGC_IAC							0x04100  /* Interrupt Assertion Count */
#define IGC_RPTHC						0x04104  /* Rx Packets To Host */
#define IGC_TLPIC						0x04148  /* EEE Tx LPI Count */
#define IGC_RLPIC						0x0414C  /* EEE Rx LPI Count */
#define IGC_HGPTC						0x04118  /* Host Good Packets Tx Count */
#define IGC_RXDMTC						0x04120  /* Rx Descriptor Minimum Threshold Count */
#define IGC_HGORCL						0x04128  /* Host Good Octets Received Count Low */
#define IGC_HGORCH						0x0412C  /* Host Good Octets Received Count High */
#define IGC_HGOTCL						0x04130  /* Host Good Octets Transmit Count Low */
#define IGC_HGOTCH						0x04134  /* Host Good Octets Transmit Count High */
#define IGC_LENERRS						0x04138  /* Length Errors Count */

#endif /* _IGC_TYPE_H_ */