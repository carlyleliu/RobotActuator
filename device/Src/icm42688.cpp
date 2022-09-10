#include <icm42688.hpp>

#include <stdio.h>

#include <inttypes.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(icm42688, 4);

#ifdef CONFIG_ICM42688_TRIGGER
static struct sensor_trigger trigger;
#endif /* CONFIG_ICM42688_TRIGGER */

static int ProcessIcm42688(const struct device *dev)
{
	struct sensor_value temperature = {0};
	struct sensor_value accel[3] = {0};
	struct sensor_value gyro[3] = {0};
    int64_t now = k_uptime_get();

	int rc = sensor_sample_fetch(dev);

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
	}
	if (rc == 0) {
		LOG_ERR("[%lld]:%g Cel  accel %f %f %f m/s/s  gyro  %f %f %f rad/s\n",
		       now,
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
static void TriggerIcm42688(const struct device *dev,
				const struct sensor_trigger *trig)
{
	int rc = ProcessIcm42688(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif /* CONFIG_ICM42688_TRIGGER */

void Icm42688Init(void)
{
    const struct device *const icm42688_dev = DEVICE_DT_GET_ONE(invensense_icm42688);

    if (!device_is_ready(icm42688_dev)) {
		LOG_ERR("Device get binding device\n");
		return;
	}

#ifdef CONFIG_ICM42688_TRIGGER
    if (IS_ENABLED(CONFIG_ICM42688_TRIGGER)) {
	    trigger = (struct sensor_trigger) {
	    	.type = SENSOR_TRIG_DATA_READY,
	    	.chan = SENSOR_CHAN_ALL,
	    };

        if (sensor_trigger_set(icm42688_dev, &trigger, TriggerIcm42688) < 0) {
	    	LOG_ERR("Cannot configure trigger\n");
	    	return;
	    }
	}
#endif // CONFIG_ICM42688_TRIGGER
    LOG_INF("Icm42688Init OK!\n");
}

void Icm42688Update(void)
{
    const struct device *const icm42688_dev = DEVICE_DT_GET_ONE(invensense_icm42688);

    while (!IS_ENABLED(CONFIG_ICM42688_TRIGGER)) {
		int rc = ProcessIcm42688(icm42688_dev);

		if (rc != 0) {
			break;
		}
		k_sleep(K_SECONDS(2));
	}
}
