#ifndef __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__

#include <motor_abstract.hpp>
#include <foc_controller.hpp>
#include <open_loop_controller.hpp>

class BldcMotor : public MotorAbstract
{
  public:
    BldcMotor();
    ~BldcMotor();

    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(uint64_t timestamp);
    void Test(void);

    FieldOrientedController& GetFocControllerHandle(void) { return foc_; };

  private:
    FieldOrientedController foc_;

  public:
    OutputPort<float> u_phase_pwm_;
    OutputPort<float> v_phase_pwm_;
    OutputPort<float> w_phase_pwm_;
};


#endif // !__MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
