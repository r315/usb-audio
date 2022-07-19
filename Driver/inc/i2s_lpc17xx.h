#ifndef _i2s_lpc17xx_h_
#define _i2s_lpc17xxh_

/* I2SDAO Bits */
#define DAO_WIDTH_8B     (0 << 0)
#define DAO_WIDTH_16B    (1 << 0)
#define DAO_WIDTH_32B    (3 << 0)
#define DAO_MONO         (1 << 2)
#define DAO_STOP         (1 << 3)
#define DAO_RESET        (1 << 4)
#define DAO_WS_SEL       (1 << 5)
#define DAO_MUTE         (1 << 15)

/* I2SDAI Bits */
#define DAI_WIDTH_8B     (0 << 0)
#define DAI_WIDTH_16B    (1 << 0)
#define DAI_WIDTH_32B    (3 << 0)
#define DAI_MONO         (1 << 2)
#define DAI_STOP         (1 << 3)
#define DAI_RESET        (1 << 4)
#define DAI_WS_SEL       (1 << 5)

/* I2SSTATE Bits */
#define STATE_IRQ        (1 << 0)
#define STATE_DMAREQ1    (1 << 1)
#define STATE_DMAREQ2    (1 << 2)
#define STATE_RX_LEVEL_pos   8
#define STATE_TX_LEVEL_pos   16

/* I2STXMODE Bits */
#define TXMODE_TX4PIN    (1 << 2)
#define TXMODE_TXMCENA   (1 << 3)

/* I2STXMODE Bits */
#define RXMODE_RX4PIN    (1 << 2)
#define RXMODE_RXMCENA   (1 << 3)

/* I2SIRQ Bits */
#define IRQ_RX_IRQ_EN    (1 << 0)
#define IRQ_TX_IRQ_EN    (1 << 1)
#define IRQ_RX_DEPTH_POS 8
#define IRQ_TX_DEPTH_POS 16
#define IRQ_RX_DEPTH_MSK (0xF << IRQ_RX_DEPTH_POS)
#define IRQ_TX_DEPTH_MSK (0xF << IRQ_TX_DEPTH_POS)

#define RXFIFO_SIZE     8
#define TXFIFO_SIZE     8


#endif
