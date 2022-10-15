#include <bldc_motor.hpp>
#include <algorithm_utils.hpp>

LOG_MODULE_REGISTER(BldcMotorControl, 3);

BldcMotor::BldcMotor() :
    MotorAbstract(),
    bldc_type_(BLDC_GIMBAL)
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
}


void BldcMotor::MotorTask(void)
{
    auto [tA, tB, tC, success] = foc_.Update();
    if (success) {
        pwm_phase_u_ = tA;
        pwm_phase_v_ = tB;
        pwm_phase_w_ = tC;
    } else {
        LOG_ERR("foc exec failed pwm[%f %f %f]\n", tA, tB, tC);
    }
}


/**
 * @brief torque close loop control
 *
 */
void BldcMotor::PositionControl(void)
{
    std::optional<float> position_measure = position_measure_.GetPresent();
    std::optional<float> velocity_measure = velocity_measure_.GetPresent();

    float position_err = target_position_ - *position_measure;

    position_err = std::clamp(position_err, -max_position_ramp_, max_position_ramp_);

    target_velocity_ = position_pid_.PIController(position_err);

    target_velocity_ = std::clamp(target_velocity_, -max_velocity_, max_velocity_);
}

/**
 * @brief torque close loop control
 *
 */
void BldcMotor::SpeedControl(void)
{
    std::optional<float> velocity_measure = velocity_measure_.GetPresent();

    if (velocity_measure > velocity_limit_tolerance * max_velocity_) {
        is_over_speed_ = true;
        return;
    }

    float velocity_err = target_velocity_ - *velocity_measure;

    velocity_err = std::clamp(velocity_err, -max_velocity_ramp_, max_velocity_ramp_);

    target_torque_ = velocity_pid_.PIDController(velocity_err);

    target_torque_ = std::clamp(target_torque_, -max_torque_, max_torque_);
}

/**
 * @brief torque close loop control
 *
 */
void BldcMotor::TorqueControl(void)
{
    float torque = direction_ * target_torque_;

        // Load setpoints from previous iteration.
    auto [id, iq] = *i_dq_target_;

    // 1% space reserved for Iq to avoid numerical issues
    id = std::clamp(id, -max_current_lim_ * 0.99f, max_current_lim_ * 0.99f);

    // Convert requested torque to current
    iq = torque / torque_constant_;

    // 2-norm clamping where Id takes priority
    float iq_lim_sqr = SQ(torque_constant_) - SQ(id);
    float iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);
    iq = std::clamp(iq, -iq_lim, iq_lim);

    if (bldc_type_ != BLDC_GIMBAL) {
        i_dq_target_ = {id, iq};
    }

    float vd = 0.0f;
    float vq = 0.0f;

    std::optional<float> phase_vel = phase_velocity_measure_.GetPresent();

    if (r_wl_ff_enable) {
        if (!phase_vel.has_value()) {
            return;
        }

        vd -= *phase_vel * phase_inductance_ * iq;
        vq += *phase_vel * phase_inductance_ * id;
        vd += phase_resistance_ * id;
        vq += phase_resistance_ * iq;
    }

    if (bemf_ff_enable_) {
        if (!phase_vel.has_value()) {
            return;
        }

        vq += *phase_vel * (2.0f / 3.0f) * (torque_constant_ / pole_pairs_);
    }

    if (bldc_type_ == BLDC_GIMBAL) {
        // reinterpret current as voltage
        v_dq_target_ = {vd + id, vq + iq};
    } else {
        v_dq_target_ = {vd, vq};
    }
}
