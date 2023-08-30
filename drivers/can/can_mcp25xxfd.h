/*
 * Copyright (c) 2020 Abram Early
 * Copyright (c) 2023 Andriy Gelman
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CAN_MICROCHIP_MCP25XXFD_H_
#define ZEPHYR_DRIVERS_CAN_MICROCHIP_MCP25XXFD_H_

#include <stdint.h>

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#define MCP25XXFD_RAM_START_ADDR 0x400
#define MCP25XXFD_RAM_SIZE       2048
#define MCP25XXFD_RAM_ALIGNMENT  4
#define MCP25XXFD_PAYLOAD_SIZE   CAN_MAX_DLEN

#define MCP25XXFD_FIFO_TYPE_TEF 0
#define MCP25XXFD_FIFO_TYPE_RX  1

#define MCP25XXFD_TEF_FIFO_ITEM_SIZE 8
#define MCP25XXFD_TX_QUEUE_ITEM_SIZE (8 + MCP25XXFD_PAYLOAD_SIZE)
#if defined(CONFIG_CAN_RX_TIMESTAMP)
#define MCP25XXFD_RX_FIFO_ITEM_SIZE  (4 + 8 + MCP25XXFD_PAYLOAD_SIZE)
#else
#define MCP25XXFD_RX_FIFO_ITEM_SIZE  (8 + MCP25XXFD_PAYLOAD_SIZE)
#endif

#define MCP25XXFD_TEF_FIFO_START_ADDR 0
#define MCP25XXFD_TEF_FIFO_ITEMS      CONFIG_CAN_MCP25XXFD_MAX_TX_QUEUE
#define MCP25XXFD_TEF_FIFO_SIZE       (MCP25XXFD_TEF_FIFO_ITEMS * MCP25XXFD_TEF_FIFO_ITEM_SIZE)

#define MCP25XXFD_TX_QUEUE_START_ADDR MCP25XXFD_TEF_FIFO_SIZE
#define MCP25XXFD_TX_QUEUE_ITEMS      CONFIG_CAN_MCP25XXFD_MAX_TX_QUEUE
#define MCP25XXFD_TX_QUEUE_SIZE       (MCP25XXFD_TX_QUEUE_ITEMS * MCP25XXFD_TX_QUEUE_ITEM_SIZE)

#define MCP25XXFD_RX_FIFO_START_ADDR (MCP25XXFD_TX_QUEUE_START_ADDR + MCP25XXFD_TX_QUEUE_SIZE)
#define MCP25XXFD_RX_FIFO_SIZE_MAX   (MCP25XXFD_RAM_SIZE - MCP25XXFD_RX_FIFO_START_ADDR)
#define MCP25XXFD_RX_FIFO_ITEMS_MAX  (MCP25XXFD_RX_FIFO_SIZE_MAX / MCP25XXFD_RX_FIFO_ITEM_SIZE)

#define MCP25XXFD_RX_FIFO_ITEMS CONFIG_CAN_MCP25XXFD_RX_FIFO_ITEMS
#define MCP25XXFD_RX_FIFO_SIZE  (MCP25XXFD_RX_FIFO_ITEMS * MCP25XXFD_RX_FIFO_ITEM_SIZE)

#define MCP25XXFD_RX_FIFO_IDX 1
#define MCP25XXFD_REG_SIZE    4

#define MCP25XXFD_CRC_POLY 0x8005
#define MCP25XXFD_CRC_SEED 0xffff

BUILD_ASSERT(MCP25XXFD_TEF_FIFO_SIZE + MCP25XXFD_TX_QUEUE_SIZE +
	     MCP25XXFD_RX_FIFO_SIZE <= MCP25XXFD_RAM_SIZE,
	     "Cannot fit FIFOs into RAM");

struct mcp25xxfd_mailbox {
	can_tx_callback_t cb;
	void *cb_arg;
};

#define MCP25XXFD_SPI_CMD_LEN       2
#define MCP25XXFD_SPI_LEN_FIELD_LEN 1
#define MCP25XXFD_SPI_CRC_LEN       2

/* MCP25XXFD Opcodes */
#define MCP25XXFD_SPI_INSTRUCTION_RESET    0x0000
#define MCP25XXFD_SPI_INSTRUCTION_WRITE    0x2000
#define MCP25XXFD_SPI_INSTRUCTION_READ     0x3000
#define MCP25XXFD_SPI_INSTRUCTION_READ_CRC 0xb000

/* MCP25XXFD Operation Modes */
#define MCP25XXFD_OPMODE_NORMAL_CANFD  0b000
#define MCP25XXFD_OPMODE_SLEEP         0b001
#define MCP25XXFD_OPMODE_INT_LOOPBACK  0b010
#define MCP25XXFD_OPMODE_LISTEN_ONLY   0b011
#define MCP25XXFD_OPMODE_CONFIGURATION 0b100
#define MCP25XXFD_OPMODE_EXT_LOOPBACK  0b101
#define MCP25XXFD_OPMODE_NORMAL_CAN2   0b110
#define MCP25XXFD_OPMODE_RESTRICTED    0b110

#define MCP25XXFD_WFT_T00FILTER 0b00
#define MCP25XXFD_WFT_T01FILTER 0b01
#define MCP25XXFD_WFT_T10FILTER 0b10
#define MCP25XXFD_WFT_T11FILTER 0b11

#define MCP25XXFD_TDCMOD_AUTO     0b10
#define MCP25XXFD_TDCMOD_MANUAL   0b01
#define MCP25XXFD_TDCMOD_DISABLED 0b00

#define MCP25XXFD_TIMING_MIN_INITIALIZER				\
	{								\
		.sjw = 1,						\
		.prop_seg = 0,						\
		.phase_seg1 = 2,					\
		.phase_seg2 = 1,					\
		.prescaler = 1,						\
	}

#define MCP25XXFD_TIMING_MAX_INITIALIZER				\
	{								\
		.sjw = 128,						\
		.prop_seg = 0,						\
		.phase_seg1 = 256,					\
		.phase_seg2 = 128,					\
		.prescaler = 256,					\
	}

#define MCP25XXFD_TIMING_DATA_MIN_INITIALIZER				\
	{								\
		.sjw = 1,						\
		.prop_seg = 0,						\
		.phase_seg1 = 1,					\
		.phase_seg2 = 1,					\
		.prescaler = 1,						\
	}

#define MCP25XXFD_TIMING_DATA_MAX_INITIALIZER				\
	{								\
		.sjw = 16,						\
		.prop_seg = 0,						\
		.phase_seg1 = 32,					\
		.phase_seg2 = 16,					\
		.prescaler = 256,					\
	}

/* MCP25XXFD Registers */

#define MCP25XXFD_REG_CON    0x000
#define MCP25XXFD_REG_CON_B2 0x002
#define MCP25XXFD_REG_CON_B3 0x003
struct mcp25xxfd_con_b0 {
	uint32_t DNCNT : 5;     /* Device Net Filter Bit Number */
	uint32_t ISOCRCEN : 1;  /* Enable ISO CRC in CAN FD Frames */
	uint32_t PXEDIS : 1;    /* Protocol Exception Event Detection Disabled */
	uint32_t res0 : 1;
} __packed;

struct mcp25xxfd_con_b1 {
	uint32_t WAKFIL : 1;    /* Enable CAN Bus Line Wake-up Filter */
	uint32_t WFT : 2;       /* Selectable Wake-up Filter Time */
	uint32_t BUSY : 1;      /* CAN Module is Busy */
	uint32_t BRSDIS : 1;    /* Bit Rate Switching Disable */
	uint32_t res1 : 3;
} __packed;

#define MCP25XXFD_CON_B2_OPMOD_MASK GENMASK(7, 5)
struct mcp25xxfd_con_b2 {
	uint32_t RTXAT : 1;     /* Restrict Retransmission Attempts */
	uint32_t ESIGM : 1;     /* Transmit ESI in Gateway Mode */
	uint32_t SERR2LOM : 1;  /* Transition to Listen Only Mode on System Error */
	uint32_t STEF : 1;      /* Store in Transmit Event FIFO */
	uint32_t TXQEN : 1;     /* Enable Transmit Queue */
	uint32_t OPMOD : 3;     /* Operation Mode Status */
} __packed;

#define MCP25XXFD_CON_B3_REQOP_MASK GENMASK(2, 0)
struct mcp25xxfd_con_b3 {
	uint32_t REQOP : 3;     /* Request Operation Mode */
	uint32_t ABAT : 1;      /* Abort All Pending Transmissions */
	uint32_t TXBWS : 4;     /* Transmit Bandwidth Sharing */
} __packed;

struct mcp25xxfd_con {
	struct mcp25xxfd_con_b0 b0;
	struct mcp25xxfd_con_b1 b1;
	struct mcp25xxfd_con_b2 b2;
	struct mcp25xxfd_con_b3 b3;
} __packed;

#define MCP25XXFD_REG_NBTCFG 0x004
struct mcp25xxfd_nbtcfg {
	uint32_t SJW : 7;       /* Synchronization Jump Width */
	uint32_t res0 : 1;
	uint32_t TSEG2 : 7;     /* Time Segment 2 (Phase Segment 2) */
	uint32_t res1 : 1;
	uint32_t TSEG1 : 8;     /* Time Segment 1 (Propagation Segment + Phase Segment 1) */
	uint32_t BRP : 8;       /* Baud Rate Prescaler */
} __packed;

#define MCP25XXFD_REG_DBTCFG 0x008
struct mcp25xxfd_dbtcfg {
	uint32_t SJW : 4;       /* Synchronization Jump Width */
	uint32_t res0 : 4;
	uint32_t TSEG2 : 4;     /* Time Segment 2 (Phase Segment 2) */
	uint32_t res1 : 4;
	uint32_t TSEG1 : 5;     /* Time Segment 1 (Propagation Segment + Phase Segment 1) */
	uint32_t res2 : 3;
	uint32_t BRP : 8;       /* Baud Rate Prescaler */
} __packed;

#define MCP25XXFD_REG_TDC  0x00C
#define MCP25XXFD_TDCO_MIN -64
#define MCP25XXFD_TDCO_MAX 63
struct mcp25xxfd_tdc {
	uint32_t TDCV : 6;      /* Transmitter Delay Compensation Value */
	uint32_t res0 : 2;
	uint32_t TDCO : 7;      /* Transmitter Delay Compensation Offset */
	uint32_t res1 : 1;
	uint32_t TDCMOD : 2;    /* Transmitter Delay Compensation Mode */
	uint32_t res2 : 6;
	uint32_t SID11EN : 1;   /* Enable 12-Bit SID in CAN FD Base Format Messages */
	uint32_t EDGFLTEN : 1;  /* Enable Edge Filtering during Bus Integration state */
} __packed;

#define MCP25XXFD_REG_TSCON 0x014
struct mcp25xxfd_tscon {
	uint32_t TBCPRE: 10; /* Time Base Counter Prescaler */
	uint32_t res0: 6;
	uint32_t TBCEN: 1; /* Time Base Counter Enable */
	uint32_t TSEOF: 1; /* 0: Beginning (See TSREF) / 1: Frame is considered valid */
	uint32_t TSRES: 1; /* Timestamp SP on FD frames) */
	uint32_t res1: 13;
};

#define MCP25XXFD_REG_INT 0x01C
struct mcp25xxfd_int_hw0 {
	uint32_t TXIF : 1;      /* Transmit FIFO Interrupt Flag */
	uint32_t RXIF : 1;      /* Receive FIFO Interrupt Flag */
	uint32_t TCBIF : 1;     /* Time Base Counter Interrupt Flag */
	uint32_t MODIF : 1;     /* Mode Change Interrupt Flag */
	uint32_t TEFIF : 1;     /* Transmit Event FIFO Interrupt Flag */
	uint32_t res0 : 3;
	uint32_t ECCIF : 1;     /* ECC Error Interrupt Flag */
	uint32_t SPICRCIF : 1;  /* SPI CRC Error Interrupt Flag */
	uint32_t TXATIF : 1;    /* Transmit Attempt Interrupt Flag */
	uint32_t RXOVIF : 1;    /* Receive FIFO Overflow Interrupt Flag */
	uint32_t SERRIF : 1;    /* System Error Interrupt Flag */
	uint32_t CERRIF : 1;    /* CAN Bus Error Interrupt Flag */
	uint32_t WAKIF : 1;     /* Bus Wake Up Interrupt Flag */
	uint32_t IVMIF : 1;     /* Invalid Message Interrupt Flag */
} __packed;

struct mcp25xxfd_int_hw1 {
	uint32_t TXIE : 1;      /* Transmit FIFO Interrupt Enable */
	uint32_t RXIE : 1;      /* Receive FIFO Interrupt Enable */
	uint32_t TBCIE : 1;     /* Time Base Counter Interrupt Enable */
	uint32_t MODIE : 1;     /* Mode Change Interrupt Enable */
	uint32_t TEFIE : 1;     /* Transmit Event FIFO Interrupt Enable */
	uint32_t res1 : 3;
	uint32_t ECCIE : 1;     /* ECC Error Interrupt Enable */
	uint32_t SPICRCIE : 1;  /* SPI CRC Error Interrupt Enable */
	uint32_t TXATIE : 1;    /* Transmit Attempt Interrupt Enable */
	uint32_t RXOVIE : 1;    /* Receive FIFO Overflow Interrupt Enable */
	uint32_t SERRIE : 1;    /* System Error Interrupt Enable */
	uint32_t CERRIE : 1;    /* CAN Bus Error Interrupt Enable */
	uint32_t WAKIE : 1;     /* Bus Wake Up Interrupt Enable */
	uint32_t IVMIE : 1;     /* Invalid Message Interrupt Enable */
} __packed;

struct mcp25xxfd_int {
	/* split into half word */
	struct mcp25xxfd_int_hw0 hw0;
	struct mcp25xxfd_int_hw1 hw1;
} __packed;

#define MCP25XXFD_REG_TREC 0x034
struct mcp25xxfd_trec {
	uint32_t REC : 8;       /* Receive Error Counter */
	uint32_t TEC : 8;       /* Transmit Error Counter */
	uint32_t EWARN : 1;     /* Transmitter or Receiver is in Error Warning State */
	uint32_t RXWARN : 1;    /* Receiver is in Error Warning State */
	uint32_t TXWARN : 1;    /* Transmitter is in Error Warning State */
	uint32_t RXBP : 1;      /* Receiver in Error Passive State */
	uint32_t TXBP : 1;      /* Transmitter in Error Passive State bit */
	uint32_t TXBO : 1;      /* Transmitter in Bus Off State */
	uint32_t res0 : 10;
} __packed;

#define MCP25XXFD_REG_BDIAG1 0x3C
struct mcp25xxfd_bdiag1 {
	uint32_t EFMSGCNT : 16;
	uint32_t NBIT0ERR : 1;
	uint32_t NBIT1ERR : 1;
	uint32_t NACKERR : 1;
	uint32_t NFORMERR : 1;
	uint32_t NSTUFERR : 1;
	uint32_t NCRCERR : 1;
	uint32_t res0 : 1;
	uint32_t TXBOERR : 1;
	uint32_t DBIT0ERR : 1;
	uint32_t DBIT1ERR : 1;
	uint32_t res1 : 1;
	uint32_t DFORMERR : 1;
	uint32_t DSTUFERR : 1;
	uint32_t DCRCERR : 1;
	uint32_t ESI : 1;
	uint32_t DLCMM : 1;
} __packed;

/* The FIFOCON, FIFOSTA, and FIFOUA registers are almost identical */
/* to their TEF and TXQ counterparts. */
#define MCP25XXFD_REG_TEFCON 0x040
#define MCP25XXFD_REG_TXQCON 0x050
#define MCP25XXFD_REG_FIFOCON(m) (MCP25XXFD_REG_TXQCON + (m) * 0xC)
#define MCP25XXFD_REG_FIFOCON_TO_STA(addr) (addr + 4)

struct mcp25xxfd_fifocon_b0 {
	uint32_t FNEIE : 1;     /* FIFO Not Full/Not Empty Interrupt Enable */
	uint32_t FHIE : 1;      /* FIFO Half Empty/Full Interrupt Enable */
	uint32_t FFIE : 1;      /* FIFO Empty/Full Interrupt Enable */
	uint32_t OVIE : 1;      /* FIFO Overflow Interrupt Enable */
	uint32_t TXATIE : 1;    /* FIFO TX Attempts Exhuasted Interrupt Enable */
	uint32_t TSEN : 1;      /* FIFO Timestamp Enable */
	uint32_t RTREN : 1;     /* FIFO Auto RTR Enable */
	uint32_t TXEN : 1;      /* FIFO Transmit Enable */
} __packed;

struct mcp25xxfd_fifocon_b1 {
	uint32_t UINC : 1;      /* FIFO Increment Head */
	uint32_t TXREQ : 1;     /* FIFO Message Send Request */
	uint32_t FRESET : 1;    /* FIFO Reset */
	uint32_t res0 : 5;
} __packed;

#define MCP25XXFD_TXAT_DISABLE_RETRANSMIT   0x0
#define MCP25XXFD_TXAT_THREE_RETRANSMIT     0x1
#define MCP25XXFD_TXAT_UNLIMITED_RETRANSMIT 0x3
struct mcp25xxfd_fifocon_b2 {
	uint32_t TXPRI : 5;     /* Transmit Priority */
	uint32_t TXAT : 2;      /* Retransmission Attempts */
	uint32_t res1 : 1;
} __packed;

struct mcp25xxfd_fifocon_b3 {
	uint32_t FSIZE : 5;   /* FIFO Size */
	uint32_t PLSIZE : 3;  /* Payload Size */
} __packed;

struct mcp25xxfd_fifocon {
	struct mcp25xxfd_fifocon_b0 b0;
	struct mcp25xxfd_fifocon_b1 b1;
	struct mcp25xxfd_fifocon_b2 b2;
	struct mcp25xxfd_fifocon_b3 b3;
} __packed;

#define MCP25XXFD_REG_TEFSTA 0x044
#define MCP25XXFD_REG_TXQSTA 0x054
#define MCP25XXFD_REG_FIFOSTA(m) (MCP25XXFD_REG_TXQSTA + (m) * 0xC)

struct mcp25xxfd_fifosta_b0 {
	uint32_t FNEIF : 1;     /* FIFO Not Full/Not Empty Interrupt Flag */
	uint32_t FHIF : 1;      /* FIFO Half Empty/Full Interrupt Flag */
	uint32_t FFIF : 1;      /* FIFO Empty/Full Interrupt Flag */
	uint32_t OVIF : 1;      /* FIFO Overflow Interrupt Flag */
	uint32_t TXATIF : 1;    /* FIFO TX Attempts Exhuasted Interrupt Flag */
	uint32_t TXERR : 1;     /* Transmission Error Status */
	uint32_t TXLARB : 1;    /* Message Lost Arbitration Status */
	uint32_t TXABT : 1;     /* Message Aborted Status */
} __packed;

struct mcp25xxfd_fifosta_b1 {
	uint32_t FIFOCI : 5;    /* FIFO Message Index */
	uint32_t res : 3;    /* FIFO Message Index */
} __packed;

struct mcp25xxfd_fifosta {
	struct mcp25xxfd_fifosta_b0 b0;
	struct mcp25xxfd_fifosta_b1 b1;
	uint16_t res;
} __packed;

#define MCP25XXFD_REG_TEFUA 0x048
#define MCP25XXFD_REG_TXQUA 0x058
#define MCP25XXFD_REG_FIFOUA(m) (MCP25XXFD_REG_TXQUA + (m) * 0xC)

struct mcp25xxfd_fiforegs {
	struct mcp25xxfd_fifocon fifocon;    /* FIFO Control Register */
	struct mcp25xxfd_fifosta fifosta;    /* FIFO Status Register */
	uint32_t ua;                    /* FIFO User Address Register */
} __packed;

struct mcp25xxfd_fifosta_ua {
	struct mcp25xxfd_fifosta fifosta;    /* FIFO Status Register */
	uint32_t ua;                    /* FIFO User Address Register */
} __packed;

#define MCP25XXFD_REG_FLTCON(m) (0x1D0 + m)
struct mcp25xxfd_fltcon {
	uint32_t FLTBP : 5;
	uint32_t res : 2;
	uint32_t FLTEN : 1;
} __packed;

#define MCP25XXFD_REG_FLTOBJ(m) (0x1F0 + ((m) * 0x8))
struct mcp25xxfd_fltobj {
	uint32_t SID : 11;
	uint32_t EID : 18;
	uint32_t SID11 : 1;
	uint32_t EXIDE : 1;
	uint32_t res0 : 1;
} __packed;

#define MCP25XXFD_REG_MASK(m) (0x1F4 + ((m) * 0x8))
struct mcp25xxfd_mask {
	uint32_t MSID : 11;
	uint32_t MEID : 18;
	uint32_t MSID11 : 1;
	uint32_t MIDE : 1;
	uint32_t res0 : 1;
} __packed;

struct mcp25xxfd_osc_b0 {
	uint32_t PLLEN : 1;     /* PLL Enable (0: Clock from XTAL, 1: Clock from 10x PLL) */
	uint32_t res0 : 1;
	uint32_t OSCDIS : 1;    /* Clock (Oscillator) Disable */
	uint32_t LPMEN : 1;     /* Low Power Mode (LPM) Enable */
	uint32_t SCLKDIV : 1;   /* System Clock Divisor (0: 1/1, 1: 1/2) */
	uint32_t CLKODIV : 2;   /* Clock Output Divisor (0: 1/1, 1: 1/2, 2: 1/4, 3: 1/10) */
	uint32_t res1 : 1;
} __packed;

struct mcp25xxfd_osc_b1 {
	uint32_t PLLRDY : 1;    /* PLL Ready (0: Not Ready, 1: Locked) */
	uint32_t res0 : 1;
	uint32_t OSCRDY : 1;    /* Clock Ready (0: Not Ready/Off, 1: Running/Stable) */
	uint32_t res1 : 1;
	uint32_t SCLKRDY : 1;   /* Synchronized SCLKDIV Bit (0: SCLKDIV 0, 1: SCLKDIV 1) */
	uint32_t res2 : 3;
} __packed;

#define MCP25XXFD_REG_OSC 0xE00
struct mcp25xxfd_osc {
	struct mcp25xxfd_osc_b0 b0;
	struct mcp25xxfd_osc_b1 b1;
	uint16_t res0;
} __packed;

#define MCP25XXFD_REG_IOCON 0xE04
struct mcp25xxfd_iocon {
	uint32_t TRIS0 : 1;     /* GPIO0 Data Direction (0: Output, 1: Input) */
	uint32_t TRIS1 : 1;     /* GPIO1 Data Direction (0: Output, 1: Input) */
	uint32_t res0 : 4;
	uint32_t XSTBYEN : 1;   /* Enable Transiever Standby Pin Control */
	uint32_t res1 : 1;
	uint32_t LAT0 : 1;      /* GPIO0 Latch (0: Low, 1: High) */
	uint32_t LAT1 : 1;      /* GPIO1 Latch (0: Low, 1: High) */
	uint32_t res2 : 6;
	uint32_t GPIO0 : 1;     /* GPIO0 Status (0: < VIL, 1: > VIH) */
	uint32_t GPIO1 : 1;     /* GPIO1 Status (0: < VIL, 1: > VIH) */
	uint32_t res3 : 6;
	uint32_t PM0 : 1;       /* GPIO0 Pin Mode (0: INT0, 1: GPIO0) */
	uint32_t PM1 : 1;       /* GPIO1 Pin Mode (0: INT1, 1: GPIO1) */
	uint32_t res4 : 2;
	uint32_t TXCANOD : 1;   /* TXCAN Drive Mode (0: Push/Pull, 1: Open Drain) */
	uint32_t SOF : 1;       /* Start-Of-Frame Signal (0: Clock on CLKO, 1: SOF on CLKO) */
	uint32_t INTOD : 1;     /* Interrupt Pins Drive Mode (0: Push/Pull, 1: Open Drain) */
	uint32_t res5 : 1;
} __packed;

/* MCP25XXFD Objects */
#define MCP25XXFD_TXQ_OBJ_HEADER_SIZE 8
struct mcp25xxfd_txobj {
	uint32_t SID : 11;
	uint32_t EID : 18;
	uint32_t SID11 : 1;
	uint32_t res0 : 2;
	uint32_t DLC : 4;       /* Data Length Code */
	uint32_t IDE : 1;       /* Indentifier Extension Flag */
	uint32_t RTR : 1;       /* Remote Transmission Request */
	uint32_t BRS : 1;       /* Bit Rate Switch Enable */
	uint32_t FDF : 1;       /* FD Frame */
	uint32_t ESI : 1;       /* Error Status Indicator */
	uint32_t SEQ : 23;
	uint8_t DATA[CAN_MAX_DLEN];
} __packed;

struct mcp25xxfd_rxobj {
	uint32_t SID: 11;
	uint32_t EID: 18;
	uint32_t SID11: 1;
	uint32_t res0: 2;
	uint32_t DLC: 4; /* Data Length Code */
	uint32_t IDE: 1; /* Indentifier Extension Flag */
	uint32_t RTR: 1; /* Remote Transmission Request */
	uint32_t BRS: 1; /* Bit Rate Switch Enable */
	uint32_t FDF: 1; /* FD Frame */
	uint32_t ESI: 1; /* Error Status Indicator */
	uint32_t res1: 2;
	uint32_t FILHIT: 5;
	uint32_t res2: 16;
#if defined(CONFIG_CAN_RX_TIMESTAMP)
	uint32_t RXMSGTS: 32;
#endif
	uint8_t DATA[CAN_MAX_DLEN];
} __packed;

struct mcp25xxfd_tefobj {
	uint32_t SID : 11;
	uint32_t EID : 18;
	uint32_t SID11 : 1;
	uint32_t res0 : 2;
	uint32_t DLC : 4;       /* Data Length Code */
	uint32_t IDE : 1;       /* Indentifier Extension Flag */
	uint32_t RTR : 1;       /* Remote Transmission Request */
	uint32_t BRS : 1;       /* Bit Rate Switch Enable */
	uint32_t FDF : 1;       /* FD Frame */
	uint32_t ESI : 1;       /* Error Status Indicator */
	uint32_t SEQ : 23;
} __packed;

#define MCP25XXFD_MAX_READ_FIFO_BUF_SIZE                                                           \
	MAX((MCP25XXFD_RX_FIFO_ITEM_SIZE * MCP25XXFD_RX_FIFO_ITEMS),                               \
	    (MCP25XXFD_TEF_FIFO_ITEM_SIZE * MCP25XXFD_TEF_FIFO_ITEMS))

#define MCP25XXFD_MAX_READ_CRC_BUF_SIZE                                                            \
	MCP25XXFD_SPI_CRC_LEN + sizeof(struct mcp25xxfd_fiforegs)

#define MCP25XXFD_SPI_BUF_SIZE                                                                     \
	MAX(MCP25XXFD_MAX_READ_FIFO_BUF_SIZE, MCP25XXFD_MAX_READ_CRC_BUF_SIZE)
#define MCP25XXFD_SPI_HEADER_LEN (MCP25XXFD_SPI_CMD_LEN + MCP25XXFD_SPI_LEN_FIELD_LEN)

struct mcp25xxfd_spi_data {
	uint8_t header[MCP25XXFD_SPI_HEADER_LEN]; /* contains spi_cmd and length field (if used) */
	uint8_t buf[MCP25XXFD_SPI_BUF_SIZE];
} __packed;

struct mcp25xxfd_fifo {
	uint32_t ram_start_addr;
	uint16_t reg_fifocon_addr;
	uint8_t capacity;
	uint8_t item_size;
	void (*msg_handler)(const struct device *dev, void *data);
};

struct mcp25xxfd_data {
	/* Interrupt Data */
	struct gpio_callback int_gpio_cb;
	struct k_thread int_thread;
	k_thread_stack_t *int_thread_stack;
	struct k_sem int_sem;

	/* General */
	enum can_state state;
	can_state_change_callback_t state_change_cb;
	void *state_change_cb_data;
	struct k_mutex mutex;

	/* TX Callback */
	struct k_sem tx_sem;
	uint32_t mailbox_usage;
	struct mcp25xxfd_mailbox mailbox[CONFIG_CAN_MCP25XXFD_MAX_TX_QUEUE];

	/* Filter Data */
	uint64_t filter_usage;
	struct can_filter filter[CONFIG_CAN_MAX_FILTER];
	can_rx_callback_t rx_cb[CONFIG_CAN_MAX_FILTER];
	void *cb_arg[CONFIG_CAN_MAX_FILTER];

	const struct device *dev;

	uint8_t sjw;
	uint8_t sjw_data;

	bool started;
	uint8_t next_mcp25xxfd_mode;
	uint8_t current_mcp25xxfd_mode;
	int tdco;

	can_mode_t mode;

	struct mcp25xxfd_spi_data spi_data;

};

struct mcp25xxfd_timing_params {
	uint8_t sjw;
	uint8_t prop_seg;
	uint8_t phase_seg1;
	uint8_t phase_seg2;
	uint32_t bus_speed;
	uint16_t sample_point;
};

struct mcp25xxfd_config {
	/* spi configuration */
	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio_dt;

	uint32_t osc_freq;

	/* IO Config */
	bool sof_on_clko;
	bool pll_enable;
	uint8_t clko_div;

	/* CAN Timing */
	struct mcp25xxfd_timing_params timing_params;
#if defined(CONFIG_CAN_FD_MODE)
	struct mcp25xxfd_timing_params timing_params_data;
#endif

	/* CAN transceiver */
	const struct device *phy;
	uint32_t max_bitrate;

	const struct device *clk_dev;
	uint8_t clk_id;

	struct mcp25xxfd_fifo rx_fifo;
	struct mcp25xxfd_fifo tef_fifo;
};

#endif /* ZEPHYR_DRIVERS_CAN_MICROCHIP_MCP25XXFD_H_ */