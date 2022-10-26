#ifndef __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__

#include <motor_abstract.hpp>
#include <foc_controller.hpp>
#include <open_loop_controller.hpp>

enum BldcMotorType
{
    BLDC_ACIM = 0,
    BLDC_GIMBAL,
    BLDC_HIGH_CURRENT,
};

class BldcMotor : public MotorAbstract
{
  public:
    BldcMotor();
    ~BldcMotor();

    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(void) final;
    void MotorRun(void) final;

    FieldOrientedController& GetFocHandle(void) {
        return foc_;
    };
    PidController& GetTorquePidHandler(void) {
        return torque_pid_;
    };
    PidController& GetVelocityPidHandler(void) {
        return velocity_pid_;
    };
    PidController& GetPositionPidHandler(void) {
        return position_pid_;
    };
    PidController& GetQAxisPidHandler(void) {
        return foc_.GetQAxisPidHandler();
    };
    PidController& GetDAxisPidHandler(void) {
        return foc_.GetDAxisPidHandler();
    };

  private:
    void TorqueControl(void);
    void SpeedControl(void);
    void PositionControl(void);

  private:
    PidController torque_pid_;
    PidController velocity_pid_;
    PidController position_pid_;

    FieldOrientedController foc_;

    enum BldcMotorType bldc_type_;

    bool r_wl_ff_enable;
    bool bemf_ff_enable_;
};


#endif // !__MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
