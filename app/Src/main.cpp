#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <adc_device.hpp>

#include <pwm_device.hpp>
#include <adc_device.hpp>

LOG_MODULE_REGISTER(Main, 7);

int main(void)
{
	printk("start main\n");

    k_msleep(3000);
    uint64_t stamp;
    struct sensor_value angle;
    const struct device *const tle5012b_dev = DEVICE_DT_GET_ONE(infineon_gmr);

    BldcMotor* bldc_ptr = new(BldcMotor);
    OpenLoopController* open_loop_control_ptr = new(OpenLoopController);

    uint16_t channel = 0;
    AdcDeviceAbstract current;
    current.SetAdcChannel(channel);
    current.SetReferenceValue(3.3);
    current.SetDividerRatio(11);

    current.Init();

    if (!device_is_ready(tle5012b_dev)) {
		LOG_ERR("Device get binding device\n");
	}

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
    uint32_t time;

    while (1) {
        k_busy_wait(1000000);
        time = k_cycle_get_32();
        printk("ticks = %d\n", time / 10000);
        // stamp = k_uptime_get();
        // open_loop_control_ptr->Reset();
        // open_loop_control_ptr->Update(stamp);
        // bldc_ptr->MotorTask(stamp);
        // pwm0_ptr->Update();
        // pwm1_ptr->Update();
        // pwm2_ptr->Update();
		// k_busy_wait(500000);

        // sensor_sample_fetch(tle5012b_dev);

        // sensor_channel_get(tle5012b_dev, SENSOR_CHAN_ALL, &angle);

        // current.Update();

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


    return 1;
}
