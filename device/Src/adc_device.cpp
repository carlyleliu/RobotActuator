#include <adc_device.hpp>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ADC, CONFIG_SENSOR_LOG_LEVEL);

static struct adc_dt_spec adc_channel[] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
};

/**
 * @brief Init adc value
 *
 */
int AdcDeviceAbstract::Init(void)
{
    int err;

    adc_spec_ = &adc_channel[adc_channel_];
    if (!device_is_ready(adc_spec_->dev)) {
		LOG_ERR("ADC controller device not ready\n");
		return -1;
	}

    err = adc_channel_setup_dt(adc_spec_);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)\n", adc_channel_, err);
		return err;
	}

    sequence_.buffer = &raw_value_.adc_16bit_value_[0];
    sequence_.buffer_size = sizeof(raw_value_.adc_16bit_value_[0]);
    // sequence_.resolution = adc_spec_->resolution;
    // sequence_.oversampling = adc_spec_->oversampling;

    adc_sequence_init_dt(adc_spec_, &sequence_);

    return 0;
}

/**
 * @brief Update adc value
 *
 */
int AdcDeviceAbstract::Update(void)
{
    int err;

    err = adc_read(adc_spec_->dev, &sequence_);
    if (err < 0) {
		LOG_ERR("Could not read (%d)\n", err);
		return err;
	}
    value_ = raw_value_.adc_16bit_value_[0] / GeRresolution() * \
                    GetReferenceValue() * GetDividerRatio();

    LOG_ERR("ADC VALUE = %d", raw_value_.adc_16bit_value_[0]);

    return 0;
}
