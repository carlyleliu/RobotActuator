/*
 * Copyright (c) 2022 Esco Medical ApS
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_
#define ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_

#include <zephyr/sys/util.h>

/* Helper macros for addressing registers in MREG1-3, see datasheet section 13 */
#define REG_MADDR_BASE			    0x00
#define REG_BANK0_OFFSET		    0x00
#define REG_MREG1_OFFSET		    (REG_MADDR_BASE)
#define REG_MREG2_OFFSET		    (REG_MADDR_BASE)
#define REG_MREG3_OFFSET		    (REG_MADDR_BASE)
#define REG_ADDRESS_MASK		    GENMASK(7, 0)
#define REG_BANK_MASK			    GENMASK(15, 8)
#define REG_SPI_READ_BIT		    BIT(7)
#define MREG_R_W_WAIT_US		    20 /* 10us, but use 20us to be on the safe side */

/* BANK 0 */
#define REG_DEVICE_CONFIG		    (REG_BANK0_OFFSET | 0x11)
#define REG_INTF_CONFIG1		    (REG_BANK0_OFFSET | 0x4d)
#define REG_INT_STATUS			    (REG_BANK0_OFFSET | 0x2d)
#define REG_WHO_AM_I			    (REG_BANK0_OFFSET | 0x75)
#define WHO_AM_I_ICM42688		    0x47
#define REG_PWR_MGMT0			    (REG_BANK0_OFFSET | 0x4e)
#define BIT_GYRO_MODE_LNM		    0x03
#define BIT_ACCEL_MODE_LNM		    0x03
#define MASK_ACCEL_MODE			    GENMASK(1, 0)
#define MASK_GYRO_MODE			    GENMASK(3, 2)
#define REG_ACCEL_DATA_X1		    (REG_BANK0_OFFSET | 0x1f)
#define REG_GYRO_DATA_X1		    (REG_BANK0_OFFSET | 0x25)
#define REG_TEMP_DATA1			    (REG_BANK0_OFFSET | 0x1d)
#define REG_INT_STATUS_DRDY		    (REG_BANK0_OFFSET | 0x39)
#define BIT_INT_STATUS_DATA_DRDY	BIT(3)
#define REG_ACCEL_CONFIG0		    (REG_BANK0_OFFSET | 0x50)
#define MASK_ACCEL_UI_FS_SEL		GENMASK(7, 5)
#define REG_GYRO_CONFIG0		    (REG_BANK0_OFFSET | 0x4f)
#define MASK_GYRO_UI_FS_SEL		    GENMASK(7, 5)
#define MASK_ACCEL_ODR			    GENMASK(3, 0)

#define BIT_ACCEL_ODR_32000		    0x01
#define BIT_ACCEL_ODR_16000		    0x02
#define BIT_ACCEL_ODR_8000		    0x03
#define BIT_ACCEL_ODR_4000		    0x04
#define BIT_ACCEL_ODR_2000		    0x05
#define BIT_ACCEL_ODR_1000		    0x06
#define BIT_ACCEL_ODR_200		    0x07
#define BIT_ACCEL_ODR_100		    0x08
#define BIT_ACCEL_ODR_50		    0x09
#define BIT_ACCEL_ODR_25		    0x0a

#define REG_GYRO_CONFIG0		    (REG_BANK0_OFFSET | 0x4f)
#define MASK_GYRO_ODR			    GENMASK(3, 0)
#define BIT_GYRO_ODR_32000		    0x01
#define BIT_GYRO_ODR_16000		    0x02
#define BIT_GYRO_ODR_8000		    0x03
#define BIT_GYRO_ODR_4000		    0x04
#define BIT_GYRO_ODR_2000		    0x05
#define BIT_GYRO_ODR_1000		    0x06
#define BIT_GYRO_ODR_200		    0x07
#define BIT_GYRO_ODR_100		    0x08
#define BIT_GYRO_ODR_50			    0x09
#define BIT_GYRO_ODR_25			    0x0a

#define BIT_ACCEL_UI_FS_16		    0x00
#define BIT_ACCEL_UI_FS_8		    0x01
#define BIT_ACCEL_UI_FS_4		    0x02
#define BIT_ACCEL_UI_FS_2		    0x03

#define BIT_GYRO_UI_FS_2000		    0x00
#define BIT_GYRO_UI_FS_1000		    0x01
#define BIT_GYRO_UI_FS_500		    0x02
#define BIT_GYRO_UI_FS_250		    0x03

#define BIT_SPI_RESET		        BIT(0)
#define BIT_STATUS_RESET_DONE_INT	BIT(4)

#define REG_INT_CONFIG			    (REG_BANK0_OFFSET | 0x14)
#define REG_INT_SOURCE0			    (REG_BANK0_OFFSET | 0x65)

#define BIT_INT_DRDY_INT1_EN	    BIT(3)
#define BIT_INT1_POLARITY		    BIT(0)
#define BIT_INT1_DRIVE_CIRCUIT	    BIT(1)

#define POWER_ON                    (0x91)
/* misc. defines */
#define MIN_ACCEL_SENS_SHIFT	    11
#define ACCEL_DATA_SIZE			    6
#define GYRO_DATA_SIZE			    6
#define TEMP_DATA_SIZE			    2
#define MCLK_POLL_INTERVAL_US	    250
#define MCLK_POLL_ATTEMPTS		    100
#define SOFT_RESET_TIME_MS		    2 /* 1ms + elbow room */

#endif /* ZEPHYR_DRIVERS_SENSOR_ICM42688_REG_H_ */
