#include <impl_adc.hpp>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ADC, LOG_LEVEL_WRN);

static struct adc_dt_spec adc_channel[] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
};

/**
 * @brief Init adc value
 *
 */
int ImplAdc::Init(void)
{
    int err = 0;

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

    adc_sequence_init_dt(adc_spec_, &sequence_);

    return err;
}

/**
 * @brief Update adc value
 *
 */
int ImplAdc::Update(void)
{
    int err = 0;

    time_ = time();

    err = adc_read(adc_spec_->dev, &sequence_);
    if (err < 0) {
		LOG_ERR("Could not read (%d)\n", err);
		return err;
	}
    value_ = raw_value_.adc_16bit_value_[0] / GeRresolution() * \
                    GetReferenceValue() * GetDividerRatio();

    LOG_INF("ADC value = %d\n", raw_value_.adc_16bit_value_[0]);

    return err;
}
