/*
 * Copyright 2015 Freescale Semiconductor, Inc.
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

#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mc_cgm_regs.h>
#include <asm/arch/mc_me_regs.h>
#include <asm/arch/clock.h>

/*
 * Computes the MFI and MFN for DFS_DVPORTn
 * The mathematical formula is the following:
 * fdfs_clckout = fdfs_clkin / ( DFS_DVPORTn[MFI] + (DFS_DVPORTn[MFN]/256) )
 * Let be MFN = 0.
 */
static u32 get_dfs_dvport_val(u32 dfs_out_freq, u32 dfs_in_freq) {
	u32 dfs_val;
	dfs_val = (dfs_in_freq/dfs_out_freq) << DFS_DVPORTn_MFI_OFFSET;
	return dfs_val;
}
/*
 * Select the clock reference for required pll.
 * pll - ARM_PLL, PERIPH_PLL, ENET_PLL, DDR_PLL, VIDEO_PLL.
 * refclk_freq - input referece clock frequency (FXOSC - 40 MHZ, FIRC - 48 MHZ)
 */
static int select_pll_source_clk( enum pll_type pll, u32 refclk_freq )
{
	u32 clk_src;
	volatile struct src * src = (struct src *                  )SRC_SOC_BASE_ADDR;

	/* select the pll clock source */
	switch( refclk_freq )
	{
			case FIRC_CLK_FREQ:
				clk_src = SRC_GPR1_FIRC_CLK_SOURCE;
				break;
			case XOSC_CLK_FREQ:
				clk_src = SRC_GPR1_XOSC_CLK_SOURCE;
				break;
		default:
				/* The clock frequency for the source clock is unknown */
				return -1;
	}

	writel( readl(&src->gpr1) | SRC_GPR1_PLL_SOURCE(pll,clk_src), &src->gpr1);

	return 0;
}
/*
 * Program the pll according to the input parameters.
 * pll - ARM_PLL, PERIPH_PLL, ENET_PLL, DDR_PLL, VIDEO_PLL.
 * refclk_freq - input referece clock frequency (FXOSC - 40 MHZ, FIRC - 48 MHZ)
 * freq - expected output frequency for PHY0
 * freq1 - expected output frequency for PHY1
 * dfs_nr - number of DFS modules for current PLL
 * dfs_freq - array with the expected frequency for DFSn
 * plldv_prediv - devider of clkfreq_ref
 * plldv_mfd - loop multiplication factor divider
 * pllfd_mfn - numerator loop multiplication factor divider
 * Please consult the PLLDIG chapter of platform manual
 * before to use this function.
 *)
 */
static int program_pll( enum pll_type pll, u32 refclk_freq, u32 freq0, u32 freq1,
						u32 dfs_nr, u32 * dfs_freq, u32 plldv_prediv, u32 plldv_mfd, u32 pllfd_mfn )
{
	u32 i, rfdphi1, rfdphi, dfs_portreset = 0, fvco;

	/*
	 * This formula is from platform reference manual (Rev. 1 Draft, D), PLLDIG chapter.
	 * The formula from Rev. 1, Draft E is wrong.
	*/
	fvco = (refclk_freq / plldv_prediv) * (plldv_mfd + pllfd_mfn/20481);

	/*
	 * VCO should have value in [ PLL_MIN_FREQ, PLL_MAX_FREQ ]. Please consult
	 * the platform DataSheet in order to determine the allowed values.
	 */

	if( fvco < PLL_MIN_FREQ || fvco > PLL_MAX_FREQ )
	{
		return -1;
	}

	if( select_pll_source_clk( pll, refclk_freq ) < 0 )
	{
		return -1;
	}

	rfdphi = fvco/freq0;

	rfdphi1 = (freq1 == 0) ? 0 : fvco/freq1;

	writel( PLLDIG_PLLDV_RFDPHI1_SET(rfdphi1) | PLLDIG_PLLDV_RFDPHI_SET(rfdphi) |
			PLLDIG_PLLDV_PREDIV_SET(plldv_prediv) | PLLDIG_PLLDV_MFD(plldv_mfd), PLLDIG_PLLDV(pll) );

	writel( readl(PLLDIG_PLLFD(pll)) | PLLDIG_PLLFD_MFN_SET(pllfd_mfn), PLLDIG_PLLFD(pll) );

	/* Only ARM_PLL, ENET_PLL and DDR_PLL */
	if( (pll == ARM_PLL) || (pll == ENET_PLL) || (pll == DDR_PLL) )
	{

		/* DFS clk enable programming */
		writel( DFS_CTRL_DLL_RESET, DFS_CTRL(pll) );

		for( i = 0; i < dfs_nr; i++ )
		{
			writel( get_dfs_dvport_val(dfs_freq[i],freq1) , DFS_DVPORTn(pll, i) );
			dfs_portreset |= ((dfs_freq[i] ? 1: 0)  << i);
		}

		writel( DFS_PORTRESET_PORTRESET_SET(dfs_portreset), DFS_PORTRESET(pll) );
	}
	return 0;

}
static void entry_to_target_mode( u32 mode )
{
	writel( mode | MC_ME_MCTL_KEY, MC_ME_MCTL );
	writel( mode | MC_ME_MCTL_INVERTEDKEY, MC_ME_MCTL );
	while( (readl(MC_ME_GS) & MC_ME_GS_S_MTRANS) != 0x00000000 );
}

static void aux_source_clk_config(uintptr_t cgm_addr, u8 ac, u32 source)
{
	#if 1
	/* select the clock source */
	writel( MC_CGM_ACn_SEL_SET(source), CGM_ACn_SC(cgm_addr, ac) );
	#endif
}
static void aux_div_clk_config(uintptr_t cgm_addr, u8 ac, u8 dc, u32 divider)
{
	#if 1
	/* set the divider */
	writel( MC_CGM_ACn_DCm_DE | MC_CGM_ACn_DCm_PREDIV(divider),
			CGM_ACn_DCm(cgm_addr, ac, dc));
	#endif
}

static void setup_sys_clocks( void )
{
	/* setup the sys clock divider for CORE_CLK (1000MHz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x0), CGM_SC_DCn(MC_CGM1_BASE_ADDR, 0) );
	/* setup the sys clock divider for CORE2_CLK (500MHz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x1), CGM_SC_DCn(MC_CGM1_BASE_ADDR, 1) );

	/* setup the sys clock divider for SYS3_CLK (266 MHz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x0), CGM_SC_DCn(MC_CGM0_BASE_ADDR, 0) );

	/* setup the sys clock divider for SYS6_CLK (133 Mhz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x1), CGM_SC_DCn(MC_CGM0_BASE_ADDR, 1) );

#if 0 /* Disable until the modules will be implemented and activated */
	/* setup the sys clock divider for GPU_CLK (600 MHz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x0), CGM_SC_DCn(MC_CGM2_BASE_ADDR, 0) );

	/* setup the sys clock divider for GPU_SHD_CLK (600 MHz)*/
	writel( MC_CGM_SC_DCn_DE | MC_CGM_SC_DCn_PREDIV(0x0), CGM_SC_DCn(MC_CGM3_BASE_ADDR, 0) );
#endif
}
static void setup_aux_clocks( void )
{
	/*
	 * setup the aux clock divider for PERI_CLK
	 * (source: PERIPH_PLL_PHI_0/5, PERI_CLK - 80 MHz)
	 */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 5, MC_CGM_ACn_SEL_PERPLLDIVX );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 5, 0, 4 );

	/* setup the aux clock divider for LIN_CLK (40MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 3, MC_CGM_ACn_SEL_PERPLLDIVX );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 3, 0, 1 );

	/* setup the aux clock divider for ENET_TIME_CLK (50MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 7, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 7, 1, 9 );

	/* setup the aux clock divider for ENET_CLK (50MHz) */
	aux_source_clk_config( MC_CGM2_BASE_ADDR, 2, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM2_BASE_ADDR, 2, 0, 9 );

	/*
	 * Disable until the modules will be implemented and activated.
	 * Please update the divider when activate the module
	 */
#if 0
	/* setup the aux clock divider for H264_DEC_CLK  (350MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 12, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 12, 0, 0 );

	/* setup the aux clock divider for H264_ENC_CLK (350MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 13, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 13, 0, 0 );

	/* setup the aux clock divider for QSPI_CLK  (416 MHz)*/
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 14, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 14, 0, 0 );
#endif

	/* setup the aux clock divider for SDHC_CLK (104 MHz). */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 15, MC_CGM_ACn_SEL_ENETPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 15, 0, 3 );

	/* setup the aux clock divider for DDR_CLK (533MHz) and APEX_SYS_CLK (266MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 8, MC_CGM_ACn_SEL_DDRPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 8, 0, 0 );

	/*
	 * Disable until the modules will be implemented and activated.
	 * Please update the divider when activate the module
	 */
#if 0

	/* setup the aux clock divider for SEQ_CLK (250MHz) and ISP_CLK (500MHz)) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 0, MC_CGM_ACn_SEL_DDRPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 0, 0, 0 );

	/* setup the aux clock divider for APEX_APU_CLK (500MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 1, MC_CGM_ACn_SEL_DDRPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 1, 0, 0 );

	/* setup the aux clock divider for MJPEG_CLK (350MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 2, MC_CGM_ACn_SEL_DDRPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 2, 0, 0 );

	/* setup the aux clock divider for DCU_AXI_CLK (300MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 9, MC_CGM_ACn_SEL_VIDEOPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 9, 0, 1 );

	/* setup the aux clock divider for DCU_PIX_CLK (150MHz) */
	aux_source_clk_config( MC_CGM0_BASE_ADDR, 9, MC_CGM_ACn_SEL_VIDEOPLL );
	aux_div_clk_config( MC_CGM0_BASE_ADDR, 9, 0, 3 );
#endif
}
static void enable_modules_clock( void )
{
	/* PIT0 */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL58 );
	/* PIT1 */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL170 );
	/* LINFLEX0 */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL83 );
	/* LINFLEX1 */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL188 );
	/* ENET */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL50 );
	/* SDHC */
	writeb( MC_ME_PCTLn_RUNPCm(0), MC_ME_PCTL93 );

	entry_to_target_mode( MC_ME_MCTL_RUN0 );
}
void clock_init(void)
{
	unsigned int arm_dfs_freq[ARM_PLL_PHI1_DFS_Nr] = {
									ARM_PLL_PHI1_DFS1_FREQ,
									ARM_PLL_PHI1_DFS2_FREQ,
									ARM_PLL_PHI1_DFS3_FREQ,
									};

	unsigned int enet_dfs_freq[ENET_PLL_PHI1_DFS_Nr] = {
									ENET_PLL_PHI1_DFS1_FREQ,
									ENET_PLL_PHI1_DFS2_FREQ,
									ENET_PLL_PHI1_DFS3_FREQ,
									ENET_PLL_PHI1_DFS4_FREQ
									};

	unsigned int ddr_dfs_freq[DDR_PLL_PHI1_DFS_Nr] = {
									DDR_PLL_PHI1_DFS1_FREQ,
									DDR_PLL_PHI1_DFS2_FREQ,
									DDR_PLL_PHI1_DFS3_FREQ,
									};

    writel( MC_ME_RUN_PCn_DRUN | MC_ME_RUN_PCn_RUN0 | MC_ME_RUN_PCn_RUN1 |
			MC_ME_RUN_PCn_RUN2 | MC_ME_RUN_PCn_RUN3,
			MC_ME_RUN_PCn(0) );

    /* turn on FXOSC */
    writel( MC_ME_RUNMODE_MC_MVRON | MC_ME_RUNMODE_MC_XOSCON |
			MC_ME_RUNMODE_MC_FIRCON, MC_ME_RUNn_MC(0) );

	entry_to_target_mode( MC_ME_MCTL_RUN0 );

	program_pll(
				ARM_PLL, XOSC_CLK_FREQ, ARM_PLL_PHI0_FREQ, ARM_PLL_PHI1_FREQ,
				ARM_PLL_PHI1_DFS_Nr, arm_dfs_freq, ARM_PLL_PLLDV_PREDIV,
				ARM_PLL_PLLDV_MFD, ARM_PLL_PLLDV_MFN
				);

	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_ARMPLL,
			MC_ME_RUNn_MC(0) );

	program_pll(
				PERIPH_PLL, XOSC_CLK_FREQ, PERIPH_PLL_PHI0_FREQ,
				PERIPH_PLL_PHI1_FREQ, PERIPH_PLL_PHI1_DFS_Nr, NULL,
				PERIPH_PLL_PLLDV_PREDIV, PERIPH_PLL_PLLDV_MFD,
				PERIPH_PLL_PLLDV_MFN
				);

	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_PERIPHPLL,
			MC_ME_RUNn_MC(0) );

	program_pll(
				ENET_PLL, XOSC_CLK_FREQ, ENET_PLL_PHI0_FREQ, ENET_PLL_PHI1_FREQ,
				ENET_PLL_PHI1_DFS_Nr, enet_dfs_freq, ENET_PLL_PLLDV_PREDIV,
				ENET_PLL_PLLDV_MFD, ENET_PLL_PLLDV_MFN
				);

	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_ENETPLL,
			MC_ME_RUNn_MC(0) );

	program_pll(
				DDR_PLL, XOSC_CLK_FREQ, DDR_PLL_PHI0_FREQ, DDR_PLL_PHI1_FREQ,
				DDR_PLL_PHI1_DFS_Nr, ddr_dfs_freq, DDR_PLL_PLLDV_PREDIV,
				DDR_PLL_PLLDV_MFD, DDR_PLL_PLLDV_MFN
				);

	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_DDRPLL,
			MC_ME_RUNn_MC(0) );

	program_pll(
				VIDEO_PLL, XOSC_CLK_FREQ, VIDEO_PLL_PHI0_FREQ,
				VIDEO_PLL_PHI1_FREQ, VIDEO_PLL_PHI1_DFS_Nr, NULL,
				VIDEO_PLL_PLLDV_PREDIV, VIDEO_PLL_PLLDV_MFD,
				VIDEO_PLL_PLLDV_MFN
				);

	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_VIDEOPLL,
			MC_ME_RUNn_MC(0) );

	setup_sys_clocks();

	setup_aux_clocks();

	/* set ARM PLL DFS 1 as SYSCLK */
	writel( readl(MC_ME_RUNn_MC(0)) | MC_ME_RUNMODE_MC_SYSCLK(0x2),
			MC_ME_RUNn_MC(0) );

	entry_to_target_mode( MC_ME_MCTL_RUN0 );

    enable_modules_clock();

}
