#ifndef __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <algorithm>

#include <component_port.hpp>
#include <pid_controller.hpp>
#include <foc_controller.hpp>

#include <fsm.hpp>

class PidController;

#define MAX_ANGLE       (-1)
#define MAX_TORQUE      (-1)
#define MAX_SPEED       (-1)
#define MAX_POSITION    (-1)

#define MAX_ANGLE_RAMP       (40)
#define MAX_TORQUE_RAMP      (60)
#define MAX_SPEED_RAMP       (1000)
#define MAX_POSITION_RAMP    (2000)

enum MotorType
{
    MOTOR_TYPE_DC = 0,
    MOTOR_TYPE_BLDC,
    MOTOR_TYPE_SERVO,
    MOTOR_TYPE_STEP,
    MOTOR_TYPE_NUM,
};

enum MotorControlType
{
    MOTOR_CONTROL_TYPE_IDLE = 0,
    MOTOR_CONTROL_TYPE_OL,
    MOTOR_CONTROL_TYPE_SPEES,
    MOTOR_CONTROL_TYPE_TORQUE,
    MOTOR_CONTROL_TYPE_POSITION,
    MOTOR_CONTROL_TYPE_NUM,
};

enum MotorControlEvent
{
    MOTOR_CONTROL_EVENT_SET_IDLE = MOTOR_CONTROL_TYPE_NUM + 1,
    MOTOR_CONTROL_EVENT_SET_OL,
    MOTOR_CONTROL_EVENT_SET_SPEES,
    MOTOR_CONTROL_EVENT_SET_TORQUE,
    MOTOR_CONTROL_EVENT_SET_POSITION,
    MOTOR_CONTROL_EVENT_NUM,
};

class MotorAbstract
{
  public:
    MotorAbstract():
        max_current_lim_(10.0f),
        torque_constant_(0.04),
        target_angle_(0),
        target_torque_(0),
        target_velocity_(0),
        target_position_(0),
        actual_angle_(0),
        actual_torque_(0),
        actual_velocity_(0),
        actual_position_(0),
        max_angle_(MAX_ANGLE),
        max_torque_(MAX_TORQUE),
        max_velocity_(MAX_SPEED),
        max_position_(MAX_POSITION),
        max_angle_ramp_(MAX_ANGLE_RAMP),
        max_torque_ramp_(MAX_TORQUE_RAMP),
        max_velocity_ramp_(MAX_SPEED_RAMP),
        max_position_ramp_(MAX_POSITION_RAMP),
        status_(0),
        fault_code_(0),
        run_(0),
        is_over_speed_(false),
        is_over_current_(false),
        is_over_temperature_(false),
        pwm_phase_u_(0.0f),
        pwm_phase_v_(0.0f),
        pwm_phase_w_(0.0f)
        {};
    virtual ~MotorAbstract(){};

  public:
    enum MotorType GetType(void) { return motor_type_; };
    enum MotorControlType GetControlType(void) { return motor_control_type_; };

    const float& GetMeasureAngle(void) { return actual_angle_; };
    const float& GetMeasureTorque(void) { return actual_torque_; };
    const float& GetMeasureVelocity(void) { return actual_velocity_; };
    const float& GetMeasurePosition(void) { return actual_position_; };

    const float& GetExpectAngle(void) { return target_angle_; };
    const float& GetExpectTorque(void) { return target_torque_; };
    const float& GetExpectVelocity(void) { return target_velocity_; };
    const float& GetExpectPosition(void) { return target_position_; };

    const uint32_t& GetStatus(void) { return status_; };
    const uint32_t& GetFaultCode(void) { return fault_code_; };
    void SetVBusMax(float vbus) { max_vbus_ = vbus; };
    float GetVBusMax(void) { return max_vbus_; };

    void SetAngle(float angle) { target_angle_ = std::clamp(angle, -max_angle_, max_angle_); };
    void SetTorque(float torque) { target_torque_ = std::clamp(torque, -max_torque_, max_torque_); };
    void SetVelocity(float velocity) { target_velocity_ = std::clamp(velocity, -max_velocity_, max_velocity_); };
    void SetPosition(float position) { target_position_ = std::clamp(position, -max_position_, max_position_); };

    float GetVBusMeasured(void) { return vbus_measured_; };
    void SetVBusMeasured(float vbus) { vbus_measured_ = vbus; };

    void SendMotorControlEvent(enum MotorControlEvent event) {fsm_.Event(event); };

  public:
    virtual void MotorStart(void) = 0;
    virtual void MotorStop(void) = 0;
    virtual void MotorTask(void) = 0;

  protected:
    enum MotorType motor_type_;
    enum MotorControlType motor_control_type_;

    float direction_ = 0.0f;// if -1 then positive torque is converted to negative Iq
    float max_current_lim_;
    float torque_constant_;
    float vbus_measured_;
    float max_vbus_;

    float velocity_limit_tolerance;

    float target_angle_;
    float target_torque_;
    float target_velocity_;
    float target_position_;

    float actual_angle_;
    float actual_torque_;
    float actual_velocity_;
    float actual_position_;

    const float max_angle_;
    const float max_torque_;
    const float max_velocity_;
    const float max_position_;

    const float max_angle_ramp_;
    const float max_torque_ramp_;
    const float max_velocity_ramp_;
    const float max_position_ramp_;

    uint64_t time_stamp_;
    uint32_t status_;
    uint32_t fault_code_;
    uint32_t run_;

    bool is_over_speed_;
    bool is_over_current_;
    bool is_over_temperature_;

    std::optional<float2D> v_dq_target_; // fed to the FOC
    std::optional<float2D> i_dq_target_; // fed to the FOC
    Fsm::StateMachine fsm_;

  public:
    /* Inputs */
    InputPort<float> position_estimate_;
    InputPort<float> position_measure_;

    InputPort<float> velocity_estimate_;
    InputPort<float> velocity_measure_;

    InputPort<float> phase_measure_;
    InputPort<float> phase_velocity_measure_;

    /* Output */
    OutputPort<float> pwm_phase_u_;
    OutputPort<float> pwm_phase_v_;
    OutputPort<float> pwm_phase_w_;
};

#endif // !__MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
