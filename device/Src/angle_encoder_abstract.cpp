#include <angle_encoder_abstract.hpp>

LOG_MODULE_REGISTER(EncoderSensor, LOG_LEVEL_WRN);

/**
* @brief  Init sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Init(void)
{
    return ImplInit();
}

/**
* @brief  DeInit sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::DeInit(void)
{
    return ImplDeInit();
}

/**
* @brief  Calibration sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Calibration(void)
{
    ImplCalibration();

    return 0;
}

/**
* @brief  align sensor
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Align(void)
{
    mechanical_angle_offset_ = ImplGetAbsoluteAngle();

    return mechanical_angle_offset_;
}

/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Update(void)
{
    if (!inited_ || !aligned_ || !calibrationed_) {
        return -EPERM;
    }

    float current_time = time();

    if (0 == time_) {
        time_ = current_time;
    }

    float dt = current_time - time_;

    mechanical_angle_measure_prev_ = mechanical_angle_measure_;

    mechanical_angle_measure_ = ImplGetAbsoluteAngle() - mechanical_angle_offset_;
    electronic_angle_measure_ = mechanical_angle_measure_ * number_of_pole_pairs_;

    int16_t delta_mechanical_angle = mechanical_angle_measure_ - mechanical_angle_measure_prev_;

	if (delta_mechanical_angle < -32768) {
		circle_counter_++;
	} else if ( delta_mechanical_angle > 32767 ) {
		circle_counter_--;
	}

    direction_ = (delta_mechanical_angle > 0) ? 1 : -1;

    mechanical_position_measure_ = (circle_counter_ + mechanical_angle_measure_ / 65536.0) * 2 * kPI_;
    normalized_angle_measure_ = (mechanical_angle_measure_ / 65536.0 ) * 2 * kPI_;
    total_angle_measure_ = normalized_angle_measure_ + circle_counter_ * 2 * kPI_;

    rpm_ = (delta_mechanical_angle / 65536.0) * dt;

    time_ = current_time;

    return 0;
}

/**
* @brief  sensor update notify to app
* @param  None
* @retval None
*/
int AngleEncoderAbstract::Notify(void)
{
    return 0;
}
