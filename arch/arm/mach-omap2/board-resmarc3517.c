/*
 * linux/arch/arm/mach-omap2/board-resmarc3517.c
 *
 * Copyright (C) 2013 Rhythm Engineering
 * Author: Kevin Selle <kevin.selle@rhythmtraffic.com>
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/pca953x.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/davinci_emac.h>
#include <linux/irq.h>
#include <linux/i2c/tsc2004.h>
#include <linux/i2c/at24.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/omap-pm.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"

#include <plat/mcspi.h>
#include <linux/spi/spi.h>
#include "linux/omap_dm_timer_bl.h"

#define AM35XX_EVM_MDIO_FREQUENCY	(1000000)

#define NAND_BLOCK_SIZE        SZ_128K

#define GPIO_BACKLIGHT_EN	35
#define BACKLIGHT_TIMER_ID	9

#define LCD_PANEL_PWR          176
#define LCD_PANEL_BKLIGHT_PWR  182
#define LCD_PANEL_PWM          181

static struct mtd_partition am3517evm_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 14*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2*(SZ_128K)
	},
	{
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40*(SZ_128K)
	},
	{
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};

static struct mdio_platform_data am3517_evm_mdio_pdata = {
	.bus_freq	= AM35XX_EVM_MDIO_FREQUENCY,
};

static struct resource am3517_mdio_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET,
		.end    = AM35XX_IPSS_EMAC_BASE + AM35XX_EMAC_MDIO_OFFSET +
			  SZ_4K - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device am3517_mdio_device = {
	.name		= "davinci_mdio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_mdio_resources),
	.resource	= am3517_mdio_resources,
	.dev.platform_data = &am3517_evm_mdio_pdata,
};

static struct emac_platform_data am3517_evm_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_emac_resources),
	.resource	= am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR |
		AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_evm_ethernet_init(struct emac_platform_data *pdata)
{
	u32 regval, mac_lo, mac_hi;

	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
	pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
	pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
	pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= am3517_enable_ethernet_int;
	pdata->interrupt_disable	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&am3517_mdio_device);
	platform_device_register(&am3517_emac_device);
	clk_add_alias(NULL, dev_name(&am3517_mdio_device.dev),
		      NULL, &am3517_emac_device.dev);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}

/* EEPROM platform data */
static struct at24_platform_data m24c08 = {
        .byte_len       = SZ_8K / 8,
        .page_size      = 16,
};

/*
 * I2C-1 (Wired to I2C_GP and I2C_PM SMARC Interfaces on LEC-3517)
 */
static struct i2c_board_info __initdata am3517evm_i2c1_boardinfo[] = {
        {
                I2C_BOARD_INFO("tps6507x", 0x48), /* Power Management IC (On-Module)*/
        },
        {
                I2C_BOARD_INFO("ssm2602", 0x1A), /* Audio Codec */
        },
};

/*
 * I2C-2 (Wired to I2C_LCD SMARC Interface on LEC-3517)
 */
static struct i2c_board_info __initdata am3517evm_i2c2_boardinfo[] = {
};

/*
 * I2C-3 (Wired to I2C_CAM SMARC Interface on LEC-3517)
 */
static struct i2c_board_info __initdata am3517evm_i2c3_boardinfo[] = {
};

static int __init am3517_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, am3517evm_i2c1_boardinfo,
			ARRAY_SIZE(am3517evm_i2c1_boardinfo));
	omap_register_i2c_bus(2, 400, am3517evm_i2c2_boardinfo,
			ARRAY_SIZE(am3517evm_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, am3517evm_i2c3_boardinfo,
                	ARRAY_SIZE(am3517evm_i2c3_boardinfo));
	return 0;
}

static int lcd_enabled;

static int rhythm_panel_enable(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PWR, 1);
	lcd_enabled = 1;

	return 0;
}

static void rhythm_panel_disable(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

static int am3517_evm_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
 
	unsigned char c;
	
	/* Temporarily disabled */
	/*
        if (level > dssdev->max_backlight_level)
                level = dssdev->max_backlight_level;

        c = ((125 * (100 - level)) / 100);
        c += get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2 ? 1 : 2;

	#define TWL_LED_PWMON   0x0
        twl_i2c_write_u8(TWL4030_MODULE_PWMA, c, TWL_LED_PWMON);
	*/

        return 0;
}

static struct omap_dss_device rhythm_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "g070y2_panel",
	.phy.dpi.data_lines 	= 24,
	.platform_enable	= rhythm_panel_enable,
	.platform_disable	= rhythm_panel_disable,
	.set_backlight          = am3517_evm_set_bl_intensity, /*tcw_debug: TODO*/
};

static struct omap_dss_device *am3517_evm_dss_devices[] = {
       &rhythm_lcd_device,
};

static struct omap_dss_board_info am3517_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_evm_dss_devices),
	.devices	= am3517_evm_dss_devices,
        .default_device = &rhythm_lcd_device,
};

static struct platform_device am3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_evm_dss_data,
	},
};

/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_evm_config[] __initdata = {
};

/* SPI */
static struct spi_board_info tcw_spi_board_info[] = {
	{
		.modalias	= "spidev",	
		.max_speed_hz	= 48000000, //48 Mbps
		.bus_num	= 1,
		.chip_select	= 0,	
		.mode = SPI_MODE_3,
	},
        {
                .modalias       = "spidev",
                .max_speed_hz   = 48000000, //48 Mbps
                .bus_num        = 2,
                .chip_select    = 0,
                .mode = SPI_MODE_3,
        },

};

static struct omap_dmtimer_bl_platform_data omap_dm_timer_bl_pdata = {
        .gpio_onoff = GPIO_BACKLIGHT_EN,
        .onoff_active_low = 0,
        .timer_id = BACKLIGHT_TIMER_ID,
        .pwm_active_low = 0,
};

static struct platform_device omap_dm_timer_bl_dev = {
        .name   = "omap-dmtimer-bl",
        .id     = -1,
        .dev    = {
                .platform_data  = &omap_dm_timer_bl_pdata,
        },
};

static struct platform_device *am3517_evm_devices[] __initdata = {
	&am3517_evm_dss_device,
	&omap_dm_timer_bl_dev,
};

static void __init am3517_evm_init_irq(void)
{
	omap_board_config = am3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(am3517_evm_config);
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 500,
};

static __init void am3517_evm_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
			| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
#else
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
#endif
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 57,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	OMAP3_MUX(SAD2D_MCAD23, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(SYS_NRESWARM, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

/*
 * HECC information
 */

#define CAN_STB         214
static void hecc_phy_control(int on)
{
        int r;

        r = gpio_request(CAN_STB, "can_stb"); /*R192 not present: STB currently not implemented*/
        if (r) {
                printk(KERN_ERR "failed to get can_stb \n");
                return;
        }

        gpio_direction_output(CAN_STB, (on==1)?0:1);
}

static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_35XX_HECC0_IRQ,
		.end	= INT_35XX_HECC0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_hecc_resources),
	.resource	= am3517_hecc_resources,
};

static struct ti_hecc_platform_data am3517_evm_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
	.transceiver_switch     = hecc_phy_control,
};

static void am3517_evm_hecc_init(struct ti_hecc_platform_data *pdata)
{
	am3517_hecc_device.dev.platform_data = pdata;
	platform_device_register(&am3517_hecc_device);
}

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -1,
		.gpio_wp	= -1,
	},
        {
                .mmc            = 2,
                .caps           = MMC_CAP_4_BIT_DATA, //8_BIT_DATA?
                .gpio_cd        = -1,
                .gpio_wp        = -1,
        },
	{}      /* Terminator */
};


static struct ehci_hcd_omap_platform_data ehci_bdata __initdata = {
        .port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
        .port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
        .port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

        .phy_reset  = true,
        .reset_gpio_port[0]  = 156, /* USB_RESET */
        .reset_gpio_port[1]  = -EINVAL,
        .reset_gpio_port[2]  = -EINVAL
};

static int sm3517_init_usb(void)
{
        int err;
	
	err = gpio_request_one(56, GPIOF_OUT_INIT_LOW, "usb2_rst");
        if (err) pr_err("SM3517: usb hub2 rst gpio request failed: %d\n", err);
	else printk(KERN_INFO "SM3517: gpio_56 registered successfully.\n");

	/*tcw_debug: reset hub u6*/
		gpio_direction_output(56, 0);
                udelay(10); /*try also: 100us*/
                gpio_direction_output(56, 1);

        return 0;
}

static void __init am3517_evm_init(void)
{
	//omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	am3517evm_i2c1_boardinfo[1].irq = gpio_to_irq(28);
	am3517_evm_i2c_init();
	platform_add_devices(am3517_evm_devices,
				ARRAY_SIZE(am3517_evm_devices));
	
	omap_serial_init();

	/* High End CAN controller */
	am3517_evm_hecc_init(&am3517_evm_hecc_pdata);

	/* USB2 */
	sm3517_init_usb();
	usb_ehci_init(&ehci_bdata);
	
	/* NAND */
	board_nand_init(am3517evm_nand_partitions,
			ARRAY_SIZE(am3517evm_nand_partitions), 0, NAND_BUSWIDTH_16);

	/*Ethernet*/
	am3517_evm_ethernet_init(&am3517_evm_emac_pdata);

	/* MUSB */
	am3517_evm_musb_init();

	/* MMC init function */
	omap2_hsmmc_init(mmc);

	/* SPI (testing) */
        spi_register_board_info(tcw_spi_board_info,
                       ARRAY_SIZE(tcw_spi_board_info));

}

MACHINE_START(RESMARC3517, "Rhythm Carrier + LEC-3517")
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= am3517_evm_init_irq,
	.init_machine	= am3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END

