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

class PwmGeneration : public Observer
{
  public:
    PwmGeneration(uint8_t array_idx);
    ~PwmGeneration();
    void Update(void) final;
    void Test(void);

  private:
    InputPort<float> pwm_input_port_;
    std::optional<float> normalize_pwm_;
    struct pwm_dt_spec* spec_;
};

#endif // ! __DEVICE_PWM_DEVICE_HPP__
