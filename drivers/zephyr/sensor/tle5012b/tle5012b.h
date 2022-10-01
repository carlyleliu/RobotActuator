#ifndef __DRIVER_SENSOR_TLE5012B_H__
#define __DRIVER_SENSOR_TLE5012B_H__

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>

#define REG_SPI_READ_BIT		    BIT(7)

/* SPI command for TLE5012 */
#define READ_STATUS			0x8001			/* 8000 */
#define READ_ANGLE_VALUE	0x8021			/* 8020 */
#define READ_SPEED_VALUE	0x8031			/* 8030 */

#define WRITE_MOD1_VALUE	0x5060			/* 0_1010_0_000110_0001 */
#define MOD1_VALUE	        0x0001

#define WRITE_MOD2_VALUE	0x5080			/* 0_1010_0_001000_0001 */
#define MOD2_VALUE	        0x0801

#define WRITE_MOD3_VALUE	0x5091			/* 0_1010_0_001001_0001 */
#define MOD3_VALUE	        0x0000

#define WRITE_MOD4_VALUE	0x50E0			/* 0_1010_0_001110_0001 */
#define MOD4_VALUE	        0x0098			/* 9bit 512 */

#define WRITE_IFAB_VALUE	0x50B1
#define IFAB_VALUE          0x000D
/* Functionality mode */
#define REFERESH_ANGLE		0

struct tle5012b_data {
    uint16_t resolution;
    int16_t angle;
    int16_t angular_velocity;
};

struct tle5012b_config {
	struct spi_dt_spec spi;
};

#endif // ! __DRIVER_SENSOR_TLE5012B_H__
