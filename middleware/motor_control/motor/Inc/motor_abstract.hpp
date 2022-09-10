#ifndef __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <algorithm>

#include <component_port.hpp>
#include <pid_controller.hpp>

class PidController;

#define MAX_ANGLE       (-1)
#define MAX_TORQUE      (-1)
#define MAX_SPEED       (-1)
#define MAX_POSITION    (-1)

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
    MOTOR_CONTROL_TYPE_OL = 0,
    MOTOR_CONTROL_TYPE_SPEES,
    MOTOR_CONTROL_TYPE_TORQUE,
    MOTOR_CONTROL_TYPE_POSITION,
    MOTOR_CONTROL_TYPE_NUM,
};

class MotorAbstract
{
  public:
    MotorAbstract():
        target_angle_(0),
        target_torque_(0),
        target_speed_(0),
        target_position_(0),
        actual_angle_(0),
        actual_torque_(0),
        actual_speed_(0),
        actual_position_(0),
        max_angle_(MAX_ANGLE),
        max_torque_(MAX_TORQUE),
        max_speed_(MAX_SPEED),
        max_position_(MAX_POSITION),
        status_(0),
        fault_code_(0),
        run_(0) {};
    ~MotorAbstract(){};

  public:
    enum MotorType GetType(void) { return motor_type_; };
    enum MotorControlType GetControlType(void) { return motor_control_type_; };

    const float& GetMeasureAngle(void) { return actual_angle_; };
    const float& GetMeasureTorque(void) { return actual_torque_; };
    const float& GetMeasureSpeed(void) { return actual_speed_; };
    const float& GetMeasurePosition(void) { return actual_position_; };

    const float& GetExpectAngle(void) { return target_angle_; };
    const float& GetExpectTorque(void) { return target_torque_; };
    const float& GetExpectSpeed(void) { return target_speed_; };
    const float& GetExpectPosition(void) { return target_position_; };

    const uint32_t& GetStatus(void) { return status_; };
    const uint32_t& GetFaultCode(void) { return fault_code_; };

    void SetAngle(float angle) { target_angle_ = std::clamp(angle, -max_angle_, max_angle_); };
    void SetTorque(float torque) { target_torque_ = std::clamp(torque, -max_torque_, max_torque_); };
    void SetSpeed(float speed) { target_speed_ = std::clamp(speed, -max_speed_, max_speed_); };
    void SetPosition(float position) { target_position_ = std::clamp(position, -max_position_, max_position_); };

  public:
    virtual void MotorInit(void) = 0;
    virtual void MotorDeInit(void) = 0;
    virtual void MotorStart(void) = 0;
    virtual void MotorStop(void) = 0;
    virtual void MotorTask(void) = 0;

  protected:
    enum MotorType motor_type_;
    enum MotorControlType motor_control_type_;

    float target_angle_;
    float target_torque_;
    float target_speed_;
    float target_position_;

    float actual_angle_;
    float actual_torque_;
    float actual_speed_;
    float actual_position_;

    const float max_angle_;
    const float max_torque_;
    const float max_speed_;
    const float max_position_;

    uint64_t time_stamp_;
    uint32_t status_;
    uint32_t fault_code_;
    uint32_t run_;
};

#endif // !__MIDDLEWARE_MOTOR_CONTROL_MOTOR_ABSTRACR_HPP__
