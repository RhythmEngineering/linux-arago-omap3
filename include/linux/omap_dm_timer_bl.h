/* linux/mfd/tps6507x.h
 *
 * Functions to access TPS65070 power management chip.
 *
 * Copyright (c) 2009 RidgeRun (todd.fischer@ridgerun.com)
 *
 *
 *  For licencing details see kernel-base/COPYING
 */

#ifndef __LINUX_OMAP_DM_TIMER_BL_H
#define __LINUX_OMAP_DM_TIMER_BL_H


	struct omap_dmtimer_bl_platform_data {
	    int gpio_onoff;
	    int onoff_active_low;
	    int timer_id;
	    int pwm_active_low;
	};

#endif /* __LINUX_OMAP_DM_TIMER_BL_H */
