#include <dc_motor.hpp>
#include <algorithm>

/** @brief Motor control construct
 *  @param None
 *  @return None
 */
DcMotor::DcMotor() :
    MotorAbstract(),
    positive_pwm_(0),
    negative_pwm_(0)
{
    motor_control_type_ = MOTOR_CONTROL_TYPE_SPEES;
    positive_pwm_ = 0;

    /* Nothiong to do
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {
    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {
    };

    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {
    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {
    };
    */

    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {
        ExecuteSpeedControl();
    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {
        MotorStop();
    };
}

/** @brief Motor control destory
 *  @param None
 *  @return None
 */
DcMotor::~DcMotor()
{
    MotorStop();
}

/** @brief Motor control Start
 *  @param None
 *  @return None
 */
void DcMotor::MotorStart(void)
{
    run_ = 1;
}

/** @brief Motor control Stop
 *  @param None
 *  @return None
 */
void DcMotor::MotorStop(void)
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
void DcMotor::MotorTask(uint64_t timestamp)
{
    if (!run_) {
        return;
    }
}

/** @brief execute speed control
 *  @param None
 *  @return None
 */
void DcMotor::ExecuteSpeedControl(void)
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
