/*
 * Copyright (c) 2018 Linaro Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <syscall_handler.h>
#include <drivers/led.h>

static inline int z_vrfy_led_blink(struct device *dev, uint32_t led,
			    uint32_t delay_on, uint32_t delay_off)
{
	Z_OOPS(Z_SYSCALL_DRIVER_LED(dev, blink));
	return z_impl_led_blink((struct device *)dev, led, delay_on,
					delay_off);
}
#include <syscalls/led_blink_mrsh.c>

static inline int z_vrfy_led_get_info(struct device *dev, uint32_t led,
				      const struct led_info **info)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_LED));
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(info, sizeof(*info)));
	return z_impl_led_get_info(dev, led, info);
}
#include <syscalls/led_get_info_mrsh.c>

static inline int z_vrfy_led_set_brightness(struct device *dev, uint32_t led,
				     uint8_t value)
{
	Z_OOPS(Z_SYSCALL_DRIVER_LED(dev, set_brightness));
	return z_impl_led_set_brightness((struct device *)dev, led, value);
}
#include <syscalls/led_set_brightness_mrsh.c>

static inline int z_impl_led_set_color(struct device *dev, uint32_t led,
				       uint8_t num_colors, const uint8_t *color)
{
	Z_OOPS(Z_SYSCALL_OBJ(dev, K_OBJ_DRIVER_LED));
	Z_OOPS(Z_SYSCALL_MEMORY_READ(color, num_colors));
	return z_impl_led_set_color(dev, led, num_colors, color);
}
#include <syscalls/led_set_color_mrsh.c>

static inline int z_vrfy_led_on(struct device *dev, uint32_t led)
{
	Z_OOPS(Z_SYSCALL_DRIVER_LED(dev, on));
	return z_impl_led_on((struct device *)dev, led);
}
#include <syscalls/led_on_mrsh.c>

static inline int z_vrfy_led_off(struct device *dev, uint32_t led)
{
	Z_OOPS(Z_SYSCALL_DRIVER_LED(dev, off));
	return z_impl_led_off((struct device *)dev, led);
}
#include <syscalls/led_off_mrsh.c>
