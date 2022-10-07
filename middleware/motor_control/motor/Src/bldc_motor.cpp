#include <bldc_motor.hpp>
#include <algorithm_utils.hpp>

LOG_MODULE_REGISTER(BldcMotorControl, 3);

BldcMotor::BldcMotor() :
    MotorAbstract(),
    u_phase_pwm_(0.0),
    v_phase_pwm_(0.0),
    w_phase_pwm_(0.0)
{
    /* enter or exit state action */
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_OL, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_OL, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_SPEES, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_SPEES, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_TORQUE, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_TORQUE, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_POSITION, STATE_INIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_EVENT_SET_POSITION, STATE_EXIT_EVENT) = [&](const Fsm::fsm_args &args) {

    };

    /* idle state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_IDLE, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* ol state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_OL, MOTOR_CONTROL_TYPE_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_OL, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_OL, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_OL, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* speed state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_SPEES, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* torque state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_TORQUE, MOTOR_CONTROL_EVENT_SET_POSITION) = [&](const Fsm::fsm_args &args) {

    };

    /* position state event to other state */
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_IDLE) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_OL) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_SPEES) = [&](const Fsm::fsm_args &args) {

    };
    fsm_.Bind(MOTOR_CONTROL_TYPE_POSITION, MOTOR_CONTROL_EVENT_SET_TORQUE) = [&](const Fsm::fsm_args &args) {

    };
}

BldcMotor::~BldcMotor()
{
    MotorStop();
}

void BldcMotor::MotorStart(void)
{
    run_ = 1;
}

void BldcMotor::MotorStop(void)
{
    run_ = 0;

    u_phase_pwm_ = 0.0;
    u_phase_pwm_.Reset();
    v_phase_pwm_ = 0.0;
    v_phase_pwm_.Reset();
    w_phase_pwm_ = 0.0;
    w_phase_pwm_.Reset();
}


void BldcMotor::MotorTask(uint64_t timestamp)
{
    foc_.Update();
}
