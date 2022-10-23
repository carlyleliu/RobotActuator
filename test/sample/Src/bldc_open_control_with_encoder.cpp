#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Main, 7);
constexpr uint32_t ktestTimes = 2000;

int main(void)
{
	printk("start main\n");

    float sensor_update_time = 0.0f;

    BldcMotor* bldc_ptr = new(BldcMotor);

    ImplPwm* pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    ImplPwm* pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    ImplPwm* pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    FieldOrientedController* foc_ptr = bldc_ptr->GetFocHandle();

    AbsoluteAngleEncoder* encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pwm0_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_w_);
    bldc_ptr->phase_measure_.ConnectTo(&encoder_ptr->measure_normalize_angle_);
    bldc_ptr->phase_velocity_measure_.ConnectTo(&encoder_ptr->measure_rpm_);

    while (1) {
        k_busy_wait(300);
        sensor_update_time = time();
        encoder_ptr->Update();

        foc_ptr->SetSensorUpdateTime(sensor_update_time);
        foc_ptr->SetVbus(12);

        bldc_ptr->MotorTask();
        pwm0_ptr->Update();
        pwm1_ptr->Update();
        pwm2_ptr->Update();
	}

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();
    bldc_ptr->phase_measure_.DisConnect();
    bldc_ptr->phase_velocity_measure_.DisConnect();

    delete(bldc_ptr);
    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    return 1;
}
