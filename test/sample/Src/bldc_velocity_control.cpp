#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(BldcOpenControl, 7);

int BldcVelocityControlSample(void)
{
	printk("start motor control sample\n");

    AngleEncoderAbstract* sensor = new (AbsoluteAngleEncoder);

    BldcMotor* bldc_ptr = new(BldcMotor);
    OpenLoopController* open_loop_control_ptr = new(OpenLoopController);

    uint16_t channel = 0;
    ImplAdc current;
    current.SetAdcChannel(channel);
    current.SetReferenceValue(3.3);
    current.SetDividerRatio(11);

    current.Init();

    ImplPwm* pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    ImplPwm* pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    ImplPwm* pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    open_loop_control_ptr->Test();

    pwm0_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->GetFocControllerHandle().pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->GetFocControllerHandle().pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&bldc_ptr->GetFocControllerHandle().pwm_phase_w_);

    bldc_ptr->GetFocControllerHandle().i_dq_target_external_.ConnectTo(&open_loop_control_ptr->i_dq_target_);
    bldc_ptr->GetFocControllerHandle().v_dq_target_external_.ConnectTo(&open_loop_control_ptr->v_dq_target_);
    bldc_ptr->GetFocControllerHandle().phase_target_external_.ConnectTo(&open_loop_control_ptr->phase_target_);
    bldc_ptr->GetFocControllerHandle().phase_velocity_target_external_.ConnectTo(&open_loop_control_ptr->phase_velocity_target_);

    while (1) {
        k_busy_wait(300);

        open_loop_control_ptr->Reset();
        open_loop_control_ptr->Update();
        bldc_ptr->MotorTask();
        pwm0_ptr->Update();
        pwm1_ptr->Update();
        pwm2_ptr->Update();

        current.Update();

	}

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();

    bldc_ptr->GetFocControllerHandle().i_dq_target_external_.DisConnect();
    bldc_ptr->GetFocControllerHandle().v_dq_target_external_.DisConnect();
    bldc_ptr->GetFocControllerHandle().phase_target_external_.DisConnect();
    bldc_ptr->GetFocControllerHandle().phase_velocity_target_external_.DisConnect();

    delete(bldc_ptr);
    delete(open_loop_control_ptr);
    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    delete(sensor);

    return 1;
}
