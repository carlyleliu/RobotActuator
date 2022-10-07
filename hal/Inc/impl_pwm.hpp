#ifndef __DEVICE_PWM_DEVICE_HPP__
#define __DEVICE_PWM_DEVICE_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

#include <observer.hpp>
#include <component_port.hpp>

using namespace DesignedPatterns;

class ImplPwm : public Observer
{
  public:
    ImplPwm() {};
    ~ImplPwm() {};
    int Init(uint8_t idx);
    int Update(void);
    void Test(void);

  private:
    struct pwm_dt_spec* spec_;
  public:
    InputPort<float> pwm_input_port_;
};

#endif // ! __DEVICE_PWM_DEVICE_HPP__
