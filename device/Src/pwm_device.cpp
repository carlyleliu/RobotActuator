#include <pwm_device.hpp>

/* define pwm channel num */
constexpr uint8_t kPwmDeviceNum = 4;

/* register logging module */
LOG_MODULE_REGISTER(PWM, CONFIG_SENSOR_LOG_LEVEL);

struct pwm_dt_spec kPwmSpec[kPwmDeviceNum] = {
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch1),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch2),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch3),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch4)
};

/**
 * @brief init pwm device struct
 * @param idx channel num
 */
void ImplPwm::Init(uint8_t idx)
{
    if (idx < kPwmDeviceNum) {
        spec_ = &kPwmSpec[idx];
        pwm_set_pulse_dt(spec_, 0);
    }
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void ImplPwm::Update(void)
{
    uint32_t pwm_pulse;

    std::optional<float> normalize_pwm = pwm_input_port_.GetAlways();

    if (normalize_pwm.has_value()) {
        pwm_pulse = (uint32_t)((*normalize_pwm) * spec_->period);
        pwm_set_pulse_dt(spec_, pwm_pulse);
        LOG_DBG("pwm_pulse[%d]\n", pwm_pulse);
    }
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void ImplPwm::Test(void)
{
    pwm_set_pulse_dt(spec_, (uint32_t)(spec_->period * 0.8));
}

