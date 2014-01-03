/*
 *  omap_dmtimer Backlight Driver
 *
 *  Copyright (c) 2013 Sergey Markov
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <plat/dmtimer.h>
#include <linux/omap_dm_timer_bl.h>

/* Flag to signal when the battery is low */
#define TIMERBL_BATTLOW       BL_CORE_DRIVER1

static uint dimming_freq = 500;

module_param_named(dimming_freq, dimming_freq, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dimming_freq, "Dimming frequency in Hz [Default: 500]");

struct omap_dmtimer_bl {
	struct backlight_device			*bldev;
	struct platform_device			*pdev;
	struct omap_dm_timer *gptimer;
	int intensity;
	u32 clock_rate;
};

static int omap_dmtimer_bl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;
	struct omap_dmtimer_bl *bl = bl_get_data(bd);
	struct omap_dmtimer_bl_platform_data *pdata;
	u32 per, cmp;

	pdata = (struct omap_dmtimer_bl_platform_data *)bl->pdev->dev.platform_data;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;
	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	if (bd->props.state & BL_CORE_FBBLANK)
		intensity = 0;
	if (bd->props.state & BL_CORE_SUSPENDED)
		intensity = 0;
	if (bd->props.state & TIMERBL_BATTLOW)
	    if (intensity > 20) 
		intensity = 20;

	if (intensity > 99) 
	    intensity = 99;

	bl->intensity = intensity;

	if (!intensity) {
	  // disable BL using GPIO
	  gpio_set_value(pdata->gpio_onoff, pdata->onoff_active_low ? 1 : 0);
	  // stop timer
	  omap_dm_timer_stop(bl->gptimer);
	} else {
	  // stop timer
	  omap_dm_timer_stop(bl->gptimer);
	  // calculate reload & compare values
	  per = bl->clock_rate / dimming_freq;
	  cmp = per * (100-intensity) / 100;
	  // configure timer compare value
	  omap_dm_timer_set_match(bl->gptimer, 1, 0xFFFFFFFFUL - cmp);
	  // configure timer reload value and start timer
	  omap_dm_timer_set_load_start(bl->gptimer, 1, 0xFFFFFFFFUL - per);
	  // Enable BL using GPIO
	  gpio_set_value(pdata->gpio_onoff, pdata->onoff_active_low ? 0 : 1);
	}

	return 0;
}

static int omap_dmtimer_bl_get_intensity(struct backlight_device *bd)
{
	struct omap_dmtimer_bl *bl = bl_get_data(bd);
	return bl->intensity;
}

static struct backlight_ops omap_dmtimer_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = omap_dmtimer_bl_get_intensity,
	.update_status  = omap_dmtimer_bl_send_intensity,
};

static int omap_dmtimer_bl_probe(struct platform_device *pdev)
{

	struct backlight_device *bd;
	struct backlight_properties props;
	struct omap_dmtimer_bl *bl;
	struct omap_dmtimer_bl_platform_data *pdata;
	struct omap_dm_timer *gptimer;

	pdata = (struct omap_dmtimer_bl_platform_data *)pdev->dev.platform_data;

	if (!pdata) {
	    dev_err(&pdev->dev, "platform_data required\n");
	    return -EIO;
	}

	if (pdata->gpio_onoff < 0 ||
	    !pdata->timer_id ||
	    pdata->timer_id < 0) {
		dev_err(&pdev->dev, "platform_data contains invalid data\n");
		return -EIO;
	}

// Acquire timer
	gptimer = omap_dm_timer_request_specific(pdata->timer_id);
	if (!gptimer) {
		dev_err(&pdev->dev, "failed to acquire timer\n");
		return -EIO;
	}

// Timer is clocked from SYSCLK
	omap_dm_timer_set_source(gptimer, OMAP_TIMER_SRC_SYS_CLK);
	
// configure PWM - default state, toggle mode, toggle events
	omap_dm_timer_set_pwm(gptimer, pdata->pwm_active_low ? 1 : 0, 1, 2);

// get clock rate
	bl = kzalloc(sizeof(struct omap_dmtimer_bl), GFP_KERNEL);
	if (!bl) {
		omap_dm_timer_free(gptimer);
		gpio_free(pdata->gpio_onoff);
		return -ENOMEM;
	}

	bl->gptimer = gptimer;
	bl->pdev = pdev;
	bl->clock_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer));

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 99;

	bd = backlight_device_register ("omap-dmtimer-bl",
		&pdev->dev, bl, &omap_dmtimer_bl_ops, &props);

	if (IS_ERR (bd)) {
	    dev_err(&pdev->dev, "Failed to register driver\n");
	    kfree(bl);
	    omap_dm_timer_free(gptimer);
	    gpio_free(pdata->gpio_onoff);
	    return -EIO;
	}

	bl->bldev = bd;

	platform_set_drvdata(pdev, bl);

	bd->props.max_brightness = 99;
	bd->props.power = FB_BLANK_UNBLANK;
	bd->props.brightness = 99;
	backlight_update_status(bd);

	printk("OMAP timer based backlight driver initialized (tmr #%u, gpio #%u, freq = %uHz)\n", pdata->timer_id, pdata->gpio_onoff, dimming_freq);
	return 0;
}

static void omap_dmtimer_bl_shutdown(struct platform_device *pdev)
{
	struct omap_dmtimer_bl *bl = platform_get_drvdata(pdev);
	struct backlight_device *bd = bl->bldev;

	printk("OMAP timer based backlight driver shutdown\n");

	bd->props.power = 0;
	bd->props.brightness = 0;
	backlight_update_status(bd);

}

static int omap_dmtimer_bl_remove(struct platform_device *pdev)
{
	struct omap_dmtimer_bl *bl = platform_get_drvdata(pdev);
	struct omap_dmtimer_bl_platform_data *pdata;

	pdata = (struct omap_dmtimer_bl_platform_data *)bl->pdev->dev.platform_data;

	omap_dmtimer_bl_shutdown(pdev);
	backlight_device_unregister(bl->bldev);
	platform_set_drvdata(pdev, NULL);
	gpio_free(pdata->gpio_onoff);
	omap_dm_timer_free(bl->gptimer);
	kfree(bl);

	printk("OMAP timer based backlight driver removed\n");

	return 0;
}

static struct platform_driver omap_dmtimer_bl_driver = {
	.probe		= omap_dmtimer_bl_probe,
	.remove		= omap_dmtimer_bl_remove,
	.shutdown	= omap_dmtimer_bl_shutdown,
	.driver		= {
		.name	= "omap-dmtimer-bl",
		.owner = THIS_MODULE,
	},
};

static int __init omap_dmtimer_bl_init(void)
{
	return platform_driver_register(&omap_dmtimer_bl_driver);
}

static void __exit omap_dmtimer_bl_exit(void)
{
	platform_driver_unregister(&omap_dmtimer_bl_driver);
}

module_init(omap_dmtimer_bl_init);
module_exit(omap_dmtimer_bl_exit);

MODULE_AUTHOR("Sergey Markov <sm@venus.ru>");
MODULE_DESCRIPTION("OMAP dual mode timer based Backlight Driver");
MODULE_LICENSE("GPL");
