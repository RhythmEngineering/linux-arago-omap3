/*
 * Driver for Advantech 7" 800x480 LCD panel (P/N: G070Y2-L01)
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/i2c/twl.h>
#include <plat/display.h>

#define TWL_PWM1_ON    0x00
#define TWL_PWM1_OFF   0x01
#define TWL_INTBR_GPBR1        0x0c
#define TWL_INTBR_PMBR1        0x0d

struct g070y2_data {
        struct backlight_device *bl;
};

static int old_brightness;

static struct omap_video_timings g070y2_timings = {
	.x_res = 800,
	.y_res = 480,
	.pixel_clock	= 29500,
	.hfp		= 104,
        .hsw            = 8,
	.hbp		= 32,
        .vfp            = 40,	
        .vsw		= 1,
	.vbp		= 2,
};

static int g070y2_bl_update_status(struct backlight_device *bl)
{
       return 0;
}

static int g070y2_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops g070y2_bl_ops = {
        .get_brightness = g070y2_bl_get_brightness,
        .update_status  = g070y2_bl_update_status,
};


static int g070y2_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;
	
	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void g070y2_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */
	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int g070y2_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = g070y2_timings;

       struct backlight_device *bl;

       bl = backlight_device_register("g070y2-bl", &dssdev->dev, dssdev,
                     &g070y2_bl_ops, &props);


       if (IS_ERR(bl)) {
               dev_err(&dssdev->dev, "failed to register backlight\n");
               return PTR_ERR(bl);
       }

       bl->props.max_brightness = 57; /* maximum brightness */
       bl->props.brightness = 57; /* initial brightness at boot-up */
       backlight_update_status(bl);

       return 0;
}

static void g070y2_panel_remove(struct omap_dss_device *dssdev)
{
       struct backlight_device *bl = platform_get_drvdata(dssdev);
       backlight_device_unregister(bl);
}

static int g070y2_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = g070y2_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void g070y2_panel_disable(struct omap_dss_device *dssdev)
{
	g070y2_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}


static int g070y2_panel_suspend(struct omap_dss_device *dssdev)
{
	g070y2_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int g070y2_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = g070y2_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void g070y2_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void g070y2_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int g070y2_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}



static struct omap_dss_driver g070y2_driver = {
	.probe		= g070y2_panel_probe,
	.remove		= g070y2_panel_remove,

	.enable		= g070y2_panel_enable,
	.disable	= g070y2_panel_disable,
	.suspend	= g070y2_panel_suspend,
	.resume		= g070y2_panel_resume,

	.set_timings	= g070y2_panel_set_timings,
	.get_timings	= g070y2_panel_get_timings,
	.check_timings	= g070y2_panel_check_timings,


	.driver         = {
		.name   = "g070y2_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init g070y2_panel_drv_init(void)
{
	return omap_dss_register_driver(&g070y2_driver);
}

static void __exit g070y2_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&g070y2_driver);
}

module_init(g070y2_panel_drv_init);
module_exit(g070y2_panel_drv_exit);
MODULE_LICENSE("GPL");

