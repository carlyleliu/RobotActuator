#include <pwm_device.hpp>

constexpr uint8_t kPwmDeviceNum = 4;

struct pwm_dt_spec kPwmSpec[kPwmDeviceNum] = {
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch1),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch2),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch3),
    PWM_DT_SPEC_GET_BY_NAME(DT_NODELABEL(pwm_spec), pwm1_ch4)
};

PwmGeneration::PwmGeneration(uint8_t array_idx)
{
    if (array_idx < kPwmDeviceNum) {
        spec_ = &kPwmSpec[array_idx];
        pwm_set_pulse_dt(spec_, 0);
    }
}

PwmGeneration::~PwmGeneration()
{
    spec_ = NULL;
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void PwmGeneration::Update(void)
{
    uint32_t pwm_pulse;

    normalize_pwm_ = pwm_input_port_.GetPresent();
    if (normalize_pwm_.has_value()) {
        pwm_pulse = (uint32_t)(*normalize_pwm_ * spec_->period);
        pwm_set_pulse_dt(spec_, pwm_pulse);
    }
}

/** @brief Update pwm device
 *  @param None
 *  @return None
 */
void PwmGeneration::Test(void)
{
    pwm_set_pulse_dt(spec_, (uint32_t)(spec_->period * 0.8));
}

