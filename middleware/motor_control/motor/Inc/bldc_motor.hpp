#ifndef __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__

#include <motor_abstract.hpp>

class BLDC_Motor : public MotorAbstract
{
  public:
    BLDC_Motor();
    ~BLDC_Motor();

    void MotorInit(void) final;
    void MotorDeInit(void) final;
    void MotorStart(void) final;
    void MotorStop(void) final;
    void MotorTask(uint64_t timestamp);
};


#endif // !__MIDDLEWARE_MOTOR_CONTROL_BLDC_MOTOR_HPP__
