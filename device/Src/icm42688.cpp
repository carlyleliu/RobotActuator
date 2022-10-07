#include <icm42688.hpp>

#include <stdio.h>

#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(ICM42688, LOG_LEVEL_WRN);

/**
 * @brief process imu data
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Process(void)
{
	struct sensor_value temperature = {0};
	struct sensor_value accel[3] = {0};
	struct sensor_value gyro[3] = {0};

    time_stamp_ = k_uptime_ticks();

	int rc = sensor_sample_fetch(dev_);

	if (rc == 0) {
		rc = sensor_channel_get(dev_, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev_, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev_, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	}
	if (rc == 0) {
        for (int idx = 0; idx < 3; idx++) {
            accel_normalized_.at(idx) = sensor_value_to_double(&accel[idx]);
            gyro_normalized_.at(idx) = sensor_value_to_double(&gyro[idx]);
        }
        temperature_ = sensor_value_to_double(&temperature);
		LOG_INF("[%lld]:%g Cel  accel %f %f %f m/s/s  gyro  %f %f %f rad/s\n",
		       time_stamp_,
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		LOG_ERR("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

#ifdef CONFIG_ICM42688_TRIGGER
int Icm42688::Trigger(void)
{
	int rc = Process(dev_);

	if (rc != 0) {
		printk("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev_, trigger_, NULL);
	}

    return rc;
}
#endif /* CONFIG_ICM42688_TRIGGER */

/**
 * @brief init imu sensor
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Init(void)
{
    dev_ = DEVICE_DT_GET_ONE(invensense_icm42688);

    if (!device_is_ready(dev_)) {
		LOG_ERR("Device get binding device\n");
		return -1;
	}

#ifdef CONFIG_ICM42688_TRIGGER
    if (IS_ENABLED(CONFIG_ICM42688_TRIGGER)) {
	    trigger_ = (struct sensor_trigger) {
	    	.type = SENSOR_TRIG_DATA_READY,
	    	.chan = SENSOR_CHAN_ALL,
	    };

        if (sensor_trigger_set(dev_, &trigger_, this->Trigger) < 0) {
	    	LOG_ERR("Cannot configure trigger\n");
	    	return -1;
	    }
	}
#endif // CONFIG_ICM42688_TRIGGER
    inited_ = 1;
    LOG_INF("Icm42688Init OK!\n");

    return 0;
}

/**
 * @brief deinit imu sensor
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::DeInit(void)
{
    dev_ = NULL;
    inited_ = 0;
}

/**
 * @brief read imu data
 *
 * @return 0 if success and -1 failed
 */
int Icm42688::Read(void)
{
    if (!IS_ENABLED(CONFIG_ICM42688_TRIGGER)) {
		return Process(dev_);
	}

    return 0;
}
