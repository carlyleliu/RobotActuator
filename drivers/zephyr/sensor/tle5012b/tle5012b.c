#include "tle5012b.h"

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT infineon_gmr

LOG_MODULE_REGISTER(TLE5012B, LOG_LEVEL_WRN);

static inline int spi_read_register(const struct spi_dt_spec *bus, uint16_t reg, uint16_t *data,
				    size_t len)
{
	uint16_t tx_buffer = reg;

	const struct spi_buf tx_buf = {
		.buf = &tx_buffer,
		.len = 2,
	};

	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 2,
		},
		{
			.buf = data,
			.len = len,
		}
	};

	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2,
	};

	return spi_transceive_dt(bus, &tx, &rx);
}

static int tle5012b_init(const struct device *dev)
{
    struct tle5012b_data *data = dev->data;
	const struct tle5012b_config *cfg = dev->config;

    if (!spi_is_ready(&cfg->spi)) {
		LOG_ERR("SPI bus is not ready");
		return -ENODEV;
	}

    data->angle = 0;
    data->angular_velocity = 0;

    return 0;
}

static int tle5012b_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    int res;
    uint16_t angle, angle_vel;
    struct tle5012b_data *data = dev->data;
	const struct tle5012b_config *cfg = dev->config;

    switch (chan)
    {
        case SENSOR_CHAN_ALL:
            res = spi_read_register(&cfg->spi, READ_ANGLE_VALUE, &angle, sizeof(angle));
            res = spi_read_register(&cfg->spi, READ_SPEED_VALUE, &angle_vel, sizeof(angle_vel));
            data->angle = angle;
            data->angular_velocity = angle_vel;
            LOG_INF("angle_vel = %d angle = %d\n", angle_vel, angle);
            break;
        default:
		    res = -ENOTSUP;
		    break;
    }

    return res;
}

static int tle5012b_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{
    int res;
    struct tle5012b_data *data = dev->data;

    switch (chan)
    {
        case SENSOR_CHAN_ALL:
            val->val1 = data->angle;
            val->val2 = data->angular_velocity;
            break;
        default:
		    res = -ENOTSUP;
		    break;
    }

    return res;
}

static const struct sensor_driver_api tle5012b_driver_api = {
	.sample_fetch = tle5012b_sample_fetch,
	.channel_get = tle5012b_channel_get,
	.attr_set = NULL,
	.attr_get = NULL,
};

/* device defaults to spi mode 0/3 support */
#define TLE5012B_SPI_CFG                                                                        \
	SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(16) | SPI_TRANSFER_MSB

#define TLE5012B_INIT(inst)                                                                     \
	static struct tle5012b_data tle5012b_driver_##inst = {                                      \
		.resolution = DT_INST_PROP(inst, resolution),                                           \
	};                                                                                          \
												                                                \
	static const struct tle5012b_config tle5012b_cfg_##inst = {                                 \
		.spi = SPI_DT_SPEC_INST_GET(inst, TLE5012B_SPI_CFG, 1U),                                \
	};                                                                                          \
												                                                \
	DEVICE_DT_INST_DEFINE(inst, tle5012b_init, NULL, &tle5012b_driver_##inst,                   \
			      &tle5012b_cfg_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,               \
			      &tle5012b_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TLE5012B_INIT)
