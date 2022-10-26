#include <motor_control_factory.hpp>

PM3505::PM3505()
{
    motor_ = new(BldcMotor);
}

PM3505::~PM3505()
{
    delete(motor_);
}

/**
 * @brief create pm3505 motor
 *
 * @return BldcMotor&
 */
BldcMotor& PM3505::Create(void)
{
    MotorConfig_t& conf = motor_->GetConfig();
    MotorControllerConfig_t& controller = motor_->GetControllerConfig();

    PidController& torque_pid = motor_->GetTorquePidHandler();
    PidController& velocity_pid = motor_->GetVelocityPidHandler();
    PidController& position_pid = motor_->GetPositionPidHandler();

    PidController& q_axis_pid = motor_->GetQAxisPidHandler();
    PidController& d_axis_pid = motor_->GetDAxisPidHandler();

    FieldOrientedController& foc = motor_->GetFocHandle();

    conf.pole_pairs_ = 11;
    conf.torque_constant_ = 0.09;
    conf.nominal_torque_ = 0.07;
    conf.stal_torque_ = 0.11;
    conf.velocity_constant_ = 74;
    conf.nominal_velocity_ = 1890;
    conf.max_velocity_ = 2160;
    conf.phase_inductance_ = 2.35;
    conf.phase_resistance_ = 5.3;
    conf.nominal_voltage_ = 12;
    conf.nominal_current_ = 0.79;
    conf.stal_current_ = 1.2;
    conf.rotor_inertia_ = 56;
    conf.max_demagnetize_tempersture_ = 120;

    controller.max_angle_ramp_ = 100;
    controller.max_torque_ramp_ = 10;
    controller.max_velocity_ramp_ = 300;
    controller.max_position_ramp_ = 300;
    controller.velocity_limit_tolerance_ = 0.99;
    controller.target_angle_ = 0;
    controller.target_torque_ = 0;
    controller.target_velocity_ = 0;
    controller.target_position_ = 0;
    controller.actual_angle_ = 0;
    controller.actual_torque_ = 0;
    controller.actual_velocity_ = 0;
    controller.actual_position_ = 0;
    controller.vbus_measured_ = 0;

    torque_pid.SetKp(10);
    torque_pid.SetKi(0.2);
    torque_pid.SetKd(0.05);

    velocity_pid.SetKp(10);
    velocity_pid.SetKi(0.2);
    velocity_pid.SetKd(0.05);

    position_pid.SetKp(10);
    position_pid.SetKi(0.2);
    position_pid.SetKd(0.05);

    q_axis_pid.SetKp(10);
    q_axis_pid.SetKi(0.2);
    q_axis_pid.SetKd(0.05);

    d_axis_pid.SetKp(10);
    d_axis_pid.SetKi(0.2);
    d_axis_pid.SetKd(0.05);

    foc.DisableCurrentControl();
    foc.SetVbus(conf.nominal_voltage_);

    return *motor_;
}
