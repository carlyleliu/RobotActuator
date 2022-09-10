#ifndef __DEVICE_LED_DEVICE_HPP__
#define __DEVICE_LED_DEVICE_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

/* extern function */
int LedInit(void);
int LedOn(uint8_t id);
int LedOff(uint8_t id);
int LedToggle(uint8_t id);
int LedTest(void);

#endif // ! __DEVICE_LED_DEVICE_HPP__
