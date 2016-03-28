/*
 * Copyright 2013-2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/siul.h>
#include <asm/arch/lpddr2.h>
#include <asm/arch/clock.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

void setup_iomux_ddr(void)
{
	lpddr2_config_iomux(DDR0);
	lpddr2_config_iomux(DDR1);

}

void ddr_phy_init(void)
{
	/* TODO: Add initialisation code for ddr phy. */
}

void ddr_ctrl_init(void)
{
	config_mmdc(0);
	config_mmdc(1);
}

int dram_init(void)
{
	setup_iomux_ddr();

	ddr_ctrl_init();

	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static void setup_iomux_uart(void)
{
	/* Muxing for linflex */
	/* Replace the magic values after bringup */

	/* set TXD - MSCR[12] PA12 */
	writel(SIUL2_UART_TXD, SIUL2_MSCRn(SIUL2_UART0_TXD_PAD));

	/* set RXD - MSCR[11] - PA11*/
	writel(SIUL2_UART_MSCR_RXD, SIUL2_MSCRn(SIUL2_UART0_MSCR_RXD_PAD));

	/* set RXD - IMCR[200] - 200 */
	writel(SIUL2_UART_IMCR_RXD, SIUL2_IMCRn(SIUL2_UART0_IMCR_RXD_PAD));
}

static void setup_iomux_enet(void)
{
	writel(0x0020c701, SIUL2_MSCRn(45));	//MDC   //PC13
	writel(0x0028c701, SIUL2_MSCRn(46));	//MDIO  //PC14
	writel(       0x2, SIUL2_MSCRn(981));

	writel(0x0008c700, SIUL2_MSCRn(47));	//RMII_CLK_REF_IP //PC15
	writel(       0x2, SIUL2_MSCRn(978));

	writel(0x0008c700, SIUL2_MSCRn(49));	//RX_D0  //PD1
	writel(       0x2, SIUL2_MSCRn(974));
	writel(0x0008c700, SIUL2_MSCRn(50));	//RX_D1  //PD2
	writel(       0x2, SIUL2_MSCRn(975));
	writel(0x0008c700, SIUL2_MSCRn(53));	//RX_DV  //PD5
	writel(       0x2, SIUL2_MSCRn(973));
	writel(0x0008c700, SIUL2_MSCRn(54));	//RX_ER  //PD6
	writel(       0x2, SIUL2_MSCRn(970));

	writel(0x0020c701, SIUL2_MSCRn(55));	//TX_D0  //PD7
	writel(0x0020c701, SIUL2_MSCRn(56));	//TX_D1  //PD8
	writel(0x0020c701, SIUL2_MSCRn(59));	//TX_EN  //PD11

#if 0
	/* reset the Ethernet controller */
	writel(0x1, 0x40032024);
	while (readl(0x40032024) & 0x1);
#endif
}

static void setup_iomux_i2c(void)
{
	/* TODO: Implement i2c iomux when it is activated. */
}

#ifdef CONFIG_SYS_USE_NAND
void setup_iomux_nfc(void)
{
	/*TODO: Implement nfc iomux when it is activated.*/
}
#endif

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg esdhc_cfg[1] = {
	{USDHC_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* eSDHC1 is always present */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	esdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_USDHC_CLK);

	/* Set iomux PADS for USDHC */

	/* PF1 pad: uSDHC clk */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(81));
	writel(0x2, SIUL2_MSCRn(902));

	/* PF2 pad: uSDHC CMD */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(82));
	writel(0x2, SIUL2_MSCRn(901));

	/* PF3 pad: uSDHC DAT0 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(83));
	writel(0x2, SIUL2_MSCRn(903));

	/* PF4 pad: uSDHC DAT1 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(84));
	writel(0x2, SIUL2_MSCRn(904));

	/* PF5 pad: uSDHC DAT2 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(85));
	writel(0x2, SIUL2_MSCRn(905));

	/* PF6 pad: uSDHC DAT3 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(86));
	writel(0x2, SIUL2_MSCRn(906));

	/* PF7 pad: uSDHC DAT4 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(87));
	writel(0x2, SIUL2_MSCRn(907));

	/* PF8 pad: uSDHC DAT5 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(88));
	writel(0x2, SIUL2_MSCRn(908));

	/* PF9 pad: uSDHC DAT6 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(89));
	writel(0x2, SIUL2_MSCRn(909));

	/* PF10 pad: uSDHC DAT7 */
	writel(SIUL2_USDHC_PAD_CTRL, SIUL2_MSCRn(90));
	writel(0x2, SIUL2_MSCRn(910));

	return fsl_esdhc_initialize(bis, &esdhc_cfg[0]);
}
#endif

static void mscm_init(void)
{
	struct mscm_ir *mscmir = (struct mscm_ir *)MSCM_BASE_ADDR;
	int i;

	for (i = 0; i < MSCM_IRSPRC_NUM; i++)
		writew(MSCM_IRSPRC_CPn_EN, &mscmir->irsprc[i]);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_early_init_f(void)
{
#if 0 /* Disable until the clocks will be adjusted to the hardware */
	clock_init();
#endif
	mscm_init();

	setup_iomux_uart();
	setup_iomux_enet();
	setup_iomux_i2c();
#ifdef CONFIG_SYS_USE_NAND
	setup_iomux_nfc();
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	return 0;
}

int checkboard(void)
{
	puts("Board: s32v234evb\n");

	return 0;
}
