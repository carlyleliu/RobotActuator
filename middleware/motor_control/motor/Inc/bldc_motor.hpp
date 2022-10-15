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
    void MotorTask(void);
    void Test(void);

    //FieldOrientedController& GetFocControllerHandle(void) { return foc_; };
    FieldOrientedController* GetFocControllerHandle(void) { return &foc_; };

  private:
    void TorqueControl(void);
    void SpeedControl(void);
    void PositionControl(void);

  private:
    FieldOrientedController foc_;
    PidController torque_pid_;
    PidController velocity_pid_;
    PidController position_pid_;

    enum BldcMotorType bldc_type_;

    bool r_wl_ff_enable;
    bool bemf_ff_enable_;

    float phase_inductance_;
    float phase_resistance_;

    int32_t pole_pairs_;
};


#endif // !__MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
