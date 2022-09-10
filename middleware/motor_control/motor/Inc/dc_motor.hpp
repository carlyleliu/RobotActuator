#ifndef __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__

#include <motor_abstract.hpp>

class WheeledChassisAbstract;

class DC_Motor : public MotorAbstract
{
  public:
    DC_Motor();
    ~DC_Motor();
    void MotorInit(void) final;
    void MotorDeInit(void) final;
    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(void) final;

  private:
    void ExecuteSpeedControl(void);

  private:
    OutputPort<float> positive_pwm_;
    OutputPort<float> negative_pwm_;
    PidController pid_controller_;
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_DC_MOTOR_HPP__
