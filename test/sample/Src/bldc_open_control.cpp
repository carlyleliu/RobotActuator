#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Sample, 7);

int MotorOpenControl(void)
{
	printk("start main\n");

    BldcMotor* bldc_ptr = new(BldcMotor);
    OpenLoopController* open_loop_control_ptr = new(OpenLoopController);

    ImplPwm* pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    ImplPwm* pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    ImplPwm* pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    FieldOrientedController* foc_ptr = bldc_ptr->GetFocControllerHandle();
    open_loop_control_ptr->SetFocController(foc_ptr);

    pwm0_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->pwm_phase_w_);

    while (1) {
        k_busy_wait(300);

        open_loop_control_ptr->Test();
        bldc_ptr->MotorTask();
        pwm0_ptr->Update();
        pwm1_ptr->Update();
        pwm2_ptr->Update();
	}

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();

    delete(bldc_ptr);
    delete(open_loop_control_ptr);
    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    return 1;
}
