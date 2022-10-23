#ifndef __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <algorithm>

#include <component_port.hpp>
#include <pid_controller.hpp>
#include <foc_controller.hpp>
#include <absolute_encoder.hpp>

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

/* motor const parameter */
typedef struct MotorConfig
{
    float torque_constant_;
    float speed_constant_;
    float phase_inductance_;
    float phase_resistance_;
    float nominal_voltage_;
    float nominal_current_;
    float stal_current_;
    float nominal_torque_;
    float stal_torque_;
    float nominal_rpm_;
    float max_rpm_;
    float rotor_inertia_;
    float max_demagnetize_tempersture_;
    uint8_t pole_pairs_;
} MotorConfig_t;

/* motor status */
typedef struct MotorStatus
{
    uint32_t fault_code_ : 16;

    uint32_t over_speed_ : 1;
    uint32_t over_current_ : 1;
    uint32_t over_temperature_ : 1;
} MotorStatus_t;

/* motor control config */
typedef struct MotorControllerConfig
{
    float max_angle_ramp_;
    float max_torque_ramp_;
    float max_velocity_ramp_;
    float max_position_ramp_;
    float velocity_limit_tolerance_;

    float target_angle_;
    float target_torque_;
    float target_velocity_;
    float target_position_;

    float actual_angle_;
    float actual_torque_;
    float actual_velocity_;
    float actual_position_;
    float vbus_measured_;
} MotorControllerConfig_t;

class MotorAbstract
{
  public:
    MotorAbstract():
        time_(0.0f),
        sensor_update_time_(0.0f),
        v_dq_target_({0.0f, 0.0f}),
        i_dq_target_({0.0f, 0.0f}),
        pwm_phase_u_(0.0f),
        pwm_phase_v_(0.0f),
        pwm_phase_w_(0.0f)
        {};
    virtual ~MotorAbstract() {};

  public:
    enum MotorType GetType(void) { return motor_type_; };
    enum MotorControlType GetControlType(void) { return motor_control_type_; };

    const float& GetMeasureAngle(void) { return motor_controller_conf_.actual_angle_; };
    const float& GetMeasureTorque(void) { return motor_controller_conf_.actual_torque_; };
    const float& GetMeasureVelocity(void) { return motor_controller_conf_.actual_velocity_; };
    const float& GetMeasurePosition(void) { return motor_controller_conf_.actual_position_; };

    const float& GetExpectAngle(void) { return motor_controller_conf_.target_angle_; };
    const float& GetExpectTorque(void) { return motor_controller_conf_.target_torque_; };
    const float& GetExpectVelocity(void) { return motor_controller_conf_.target_velocity_; };
    const float& GetExpectPosition(void) { return motor_controller_conf_.target_position_; };

    const uint32_t GetFaultCode(void) { return motor_status_.fault_code_; };

    void SetAngle(float angle) { motor_controller_conf_.target_angle_ = angle; };
    void SetTorque(float torque) { motor_controller_conf_.target_torque_ = std::clamp(torque, -motor_conf_.stal_torque_, motor_conf_.stal_torque_); };
    void SetVelocity(float velocity) { motor_controller_conf_.target_velocity_ = std::clamp(velocity, -motor_conf_.max_rpm_, motor_conf_.max_rpm_); };
    void SetPosition(float position) { motor_controller_conf_.target_position_ = position; };

    float GetVBusMeasured(void) { return motor_controller_conf_.vbus_measured_; };
    void SetVBusMeasured(float vbus) { motor_controller_conf_.vbus_measured_ = vbus; };

    void SendMotorControlEvent(enum MotorControlEvent event) {fsm_.Event(event); };

  public:
    virtual void MotorStart(void) = 0;
    virtual void MotorStop(void) = 0;
    virtual void MotorRun(void) = 0;
    virtual void MotorTask(void) = 0;

  protected:
    enum MotorType motor_type_;
    enum MotorControlType motor_control_type_;
    MotorConfig_t motor_conf_;
    MotorStatus_t motor_status_;
    MotorControllerConfig_t motor_controller_conf_;

    float time_;
    float sensor_update_time_;

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
