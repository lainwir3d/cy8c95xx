/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_I2C_CY8C95XX_H
#define __LINUX_I2C_CY8C95XX_H

#define CONFIG_GPIO_CY8C95XX_IRQ
#define FAST_INTERRUPT
#define IRQ_VALUE_PREFETCH
//#define DEBUG

/* platform data for the MAX732x 8/16-bit I/O expander driver */

struct cy8c95xx_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
};
#endif /* __LINUX_I2C_CY8C95XX_H */
