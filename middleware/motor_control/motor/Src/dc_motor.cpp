#include <dc_motor.hpp>
#include <algorithm>

/** @brief Motor control Init
 *  @param None
 *  @return None
 */
void DC_Motor::MotorInit(void)
{
    positive_pwm_ = 0;
}

/** @brief Motor control DeInit
 *  @param None
 *  @return None
 */
void DC_Motor::MotorDeInit(void)
{
    run_ = 0;
    positive_pwm_ = 0;
    positive_pwm_.Reset();
    negative_pwm_ = 0;
    negative_pwm_.Reset();
}

/** @brief Motor control Start
 *  @param None
 *  @return None
 */
void DC_Motor::MotorStart(void)
{
    run_ = 1;
}

/** @brief Motor control Stop
 *  @param None
 *  @return None
 */
void DC_Motor::MotorStop(void)
{
    run_ = 0;
    positive_pwm_ = 0;
    positive_pwm_.Reset();
    negative_pwm_ = 0;
    negative_pwm_.Reset();
}

/** @brief Motor control Task
 *  @param None
 *  @return None
 */
void DC_Motor::MotorTask(void)
{
    if (!run_) {
        return;
    }

    /* only support MOTOR_CONTROL_TYPE_SPEES and MOTOR_CONTROL_TYPE_POSITIONcontrol mode */
    if(MOTOR_CONTROL_TYPE_SPEES == motor_control_type_) {
        ExecuteSpeedControl();
    } else if (MOTOR_CONTROL_TYPE_POSITION == motor_control_type_) {
        MotorStop();
    } else {
        MotorStop();
    }
}

/** @brief execute speed control
 *  @param None
 *  @return None
 */
void DC_Motor::ExecuteSpeedControl(void)
{
    constexpr float one = 1.0f;
    float delta_speed = target_speed_ - actual_speed_;
    float out = pid_controller_.PIDController(delta_speed);
    if (out > 0) {
        positive_pwm_ = std::clamp(out, -one, one);
        negative_pwm_ = 0;
    } else {
        positive_pwm_ = 0;
        negative_pwm_ = std::clamp(-out, -one, one);
    }
}
