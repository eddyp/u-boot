/*
 * (C) Copyright 2013-2016 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#ifdef CONFIG_DM
#include <dm.h>
#include <errno.h>
#endif
#include <watchdog.h>
#include <asm/io.h>
#include <serial.h>
#include <linux/compiler.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>

#define US1_TDRE            (1 << 7)
#define US1_RDRF            (1 << 5)
#define UC2_TE              (1 << 3)
#define LINCR1_INIT         (1 << 0)
#define LINCR1_MME          (1 << 4)
#define LINCR1_BF           (1 << 7)
#define LINSR_LINS_INITMODE (0x00001000)
#define LINSR_LINS_MASK     (0x0000F000)
#define UARTCR_UART         (1 << 0)
#define UARTCR_WL0          (1 << 1)
#define UARTCR_PCE          (1 << 2)
#define UARTCR_PC0          (1 << 3)
#define UARTCR_TXEN         (1 << 4)
#define UARTCR_RXEN         (1 << 5)
#define UARTCR_PC1          (1 << 6)
#define UARTCR_TFBM         (1 << 8)
#define UARTCR_RFBM         (1 << 9)
#define UARTSR_DTF          (1 << 1)
#define UARTSR_DRF          (1 << 2)
#define UARTSR_RFE          (1 << 2)
#define UARTSR_RFNE         (1 << 4)
#define UARTSR_RMB          (1 << 9)




DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_DM_SERIAL
struct linflex_fsl *lfbase = (struct linflex_fsl *)LINFLEXUART_BASE;
#endif

static void _linflex_serial_setbrg(struct linflex_fsl *base, int baudrate)
{
    u32 clk = mxc_get_clock(MXC_UART_CLK);
    u32 ibr, fbr;

	if (!baudrate)
		baudrate = CONFIG_BAUDRATE;

    ibr = (u32)(clk / (16 * gd->baudrate));
    fbr = (u32)(clk % (16* gd->baudrate))*16;

    __raw_writel(ibr, &base->linibrr);
    __raw_writel(fbr, &base->linfbrr);

}

static int _linflex_serial_getc(struct linflex_fsl *base)
{
#ifndef CONFIG_DM_SERIAL
	while ((__raw_readl(&base->uartsr) & UARTSR_RFE) == UARTSR_RFE) {}

#else
	if ((__raw_readl(&base->uartsr) & UARTSR_RFE))
		return -EAGAIN;

#endif

	return __raw_readb(&base->bdrm);
}

static int _linflex_serial_putc(struct linflex_fsl *base, const char c)
{
	#ifndef CONFIG_DM_SERIAL
	if (c == '\n')
		serial_putc('\r');

	/* wait for Tx FIFO not full */
	while ((__raw_readb(&base->uartsr) & UARTSR_DTF)) {}
	#else
	if ((__raw_readb(&base->uartsr) & UARTSR_DTF))
		return -EAGAIN;
	#endif

	__raw_writeb(c, &base->bdrl);

	if ((__raw_readb(&base->uartsr) & UARTSR_RFE) == UARTSR_RFE)
		return 0;

	return 1;
}

/*
 * Initialise the serial port with the given baudrate. The settings
 * are always 8 data bits, no parity, 1 stop bit, no start bits.
 */
static int _linflex_serial_init(struct linflex_fsl *base)
{
    volatile u32 ctrl;

    /* set the Linflex in master mode amd activate by-pass filter */
    ctrl = LINCR1_BF | LINCR1_MME;
    __raw_writel(ctrl, &base->lincr1);

    /* init mode */
    ctrl |= LINCR1_INIT;
    __raw_writel(ctrl, &base->lincr1);

    /* waiting for init mode entry - TODO: add a timeout */
    while((__raw_readl(&base->linsr) & LINSR_LINS_MASK) != LINSR_LINS_INITMODE){}


    /* set UART bit to allow writing other bits */
    __raw_writel(UARTCR_UART, &base->uartcr);
    /* provide data bits, parity, stop bit, etc */
    serial_setbrg();
    /* 8 bit data, no parity, Tx and Rx enabled, UART mode */
    __raw_writel(UARTCR_PC1 | UARTCR_RXEN | UARTCR_TXEN | UARTCR_PC0
                 | UARTCR_WL0 | UARTCR_UART | UARTCR_RFBM | UARTCR_TFBM, &base->uartcr);

    ctrl = __raw_readl(&base->lincr1);
    ctrl &= ~LINCR1_INIT;
    __raw_writel(ctrl, &base->lincr1); /* end init mode */

    return 0;
}

#ifndef CONFIG_DM_SERIAL
/*
 * Test whether a character is in the RX buffer
 */
static int _linflex_serial_tstc(void)
{
	if ((__raw_readb(&lfbase->uartsr) & UARTSR_RFE) == UARTSR_RFE)
		return 0;

    return 1;
}

static int nondm_linflex_serial_init(void)
{
	return _linflex_serial_init(lfbase);
}

static void nondm_linflex_serial_setbrg(void)
{
	_linflex_serial_setbrg(lfbase, CONFIG_BAUDRATE);
}

static void nondm_linflex_serial_putc(const char c)
{
	_linflex_serial_putc(lfbase,c);
}

static int nondm_linflex_serial_getc(void)
{
	return _linflex_serial_getc(lfbase);
}

static struct serial_device linflex_serial_drv = {
	.name = "linflex_serial",
	.start = nondm_linflex_serial_init,
	.stop = NULL,
	.setbrg = nondm_linflex_serial_setbrg,
	.putc = nondm_linflex_serial_putc,
	.puts = default_serial_puts,
	.getc = nondm_linflex_serial_getc,
	.tstc = _linflex_serial_tstc,
};

void linflex_serial_initialize(void)
{
    serial_register(&linflex_serial_drv);
}

__weak struct serial_device *default_serial_console(void)
{
    return &linflex_serial_drv;
}
#endif /* ifndef(CONFIG_DM_SERIAL) */

#ifdef CONFIG_DM_SERIAL

struct linflex_serial_platdata {
	struct linflex_fsl *base_addr;
	u8 port_id; /* do we need this? */
};

struct linflex_serial_priv {
	struct linflex_fsl *lfuart;
};

int linflex_serial_setbrg(struct udevice *dev, int baudrate)
{
	struct linflex_serial_priv *priv = dev_get_priv(dev);

	_linflex_serial_setbrg(priv->lfuart, baudrate);

	return 0;
}

static int linflex_serial_getc(struct udevice *dev)
{
	struct linflex_serial_priv *priv = dev_get_priv(dev);

	return _linflex_serial_getc(priv->lfuart);
}

static int linflex_serial_putc(struct udevice *dev, const char ch)
{

	struct linflex_serial_priv *priv = dev_get_priv(dev);

	return _linflex_serial_putc(priv->lfuart, ch);
}

static int linflex_serial_pending(struct udevice *dev, bool input)
{
	struct linflex_serial_priv *priv = dev_get_priv(dev);
	uint32_t uartsr = __raw_readl(&priv->lfuart->uartsr);

	if (input)
		return ((uartsr & UARTSR_DRF) && (uartsr & UARTSR_RMB)) ? 1 : 0;
	else
		return uartsr & UARTSR_DTF ? 0 : 1;
}

static void linflex_serial_init_internal(struct linflex_fsl *lfuart)
{
	_linflex_serial_init(lfuart);
	_linflex_serial_setbrg(lfuart, CONFIG_BAUDRATE);
	return;
}

static int linflex_serial_probe(struct udevice *dev)
{
	struct linflex_serial_platdata *plat = dev->platdata;
	struct linflex_serial_priv *priv = dev_get_priv(dev);

	priv->lfuart = (struct linflex_fsl *)plat->base_addr;
	linflex_serial_init_internal(priv->lfuart);

	return 0;
}

static const struct dm_serial_ops linflex_serial_ops = {
	.putc = linflex_serial_putc,
	.pending = linflex_serial_pending,
	.getc = linflex_serial_getc,
	.setbrg = linflex_serial_setbrg,
};

U_BOOT_DRIVER(serial_linflex) = {
	.name	= "serial_linflex",
	.id	= UCLASS_SERIAL,
	.probe = linflex_serial_probe,
	.ops	= &linflex_serial_ops,
	.flags = DM_FLAG_PRE_RELOC,
	.priv_auto_alloc_size	= sizeof(struct linflex_serial_priv),
};
#endif

#ifdef CONFIG_DEBUG_UART_LINFLEXUART

#include <debug_uart.h>


static inline void _debug_uart_init(void)
{
	struct linflex_fsl *base = (struct linflex_fsl *)CONFIG_DEBUG_UART_BASE;

	linflex_serial_init_internal(base);
}

static inline void _debug_uart_putc(int ch)
{
	struct linflex_fsl *base = (struct linflex_fsl *)CONFIG_DEBUG_UART_BASE;

	/* XXX: Is this OK? Should this use the non-DM version? */
	_linflex_serial_putc(base, ch);
}

#endif /* CONFIG_DEBUG_UART_LINFLEXUART */
