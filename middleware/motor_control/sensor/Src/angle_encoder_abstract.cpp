#include <angle_encoder_abstract.hpp>

/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  None
* @retval None
*/
void AngleEncoderAbstract::UpdateData(void)
{
    if (!inited_ || !aligned_ || !calibrationed_) {
        return;
    }

    mechanical_angle_measure_prev_ = mechanical_angle_measure_;

    mechanical_angle_measure_ = GetAbsoluteAngle() - mechanical_angle_offset_;
    electronic_angle_measure_ = mechanical_angle_measure_ * number_of_pole_pairs_;

    int16_t delta_mechanical_angle = mechanical_angle_measure_ - mechanical_angle_measure_prev_;

	if (delta_mechanical_angle < -32768) {
		circle_counter_++;
	} else if ( delta_mechanical_angle > 32767 ) {
		circle_counter_--;
	}
}
