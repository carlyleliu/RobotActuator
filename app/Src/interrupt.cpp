#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/counter.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>
#include <motor_control_factory.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Main, 7);

#define PERIOD              1000
#define ALARM_CHANNEL_ID    0

struct counter_alarm_cfg alarm_cfg;

#define COUNTER DT_NODELABEL(counter2)

PM3505 pm3505;
ImplPwm* pwm0_ptr = nullptr;
ImplPwm* pwm1_ptr = nullptr;
ImplPwm* pwm2_ptr = nullptr;
AbsoluteAngleEncoder* encoder_ptr = nullptr;

extern int8_t run_control;

void MotorControlUpdate(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    encoder_ptr->Update();

    pm3505_bldc.MotorTask();
    pwm0_ptr->Update();
    pwm1_ptr->Update();
    pwm2_ptr->Update();
}

static void HighFreqTask(const struct device *counter_dev,
				      uint8_t chan_id, uint32_t ticks,
				      void *user_data)
{
	struct counter_alarm_cfg *config = (struct counter_alarm_cfg*)user_data;
	int err;

	/* Set a new alarm with a double length duration */
	config->ticks = config->ticks;

	err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,
					(struct counter_alarm_cfg*)user_data);

	if (err != 0) {
		printk("Alarm could not be set\n");
	}

    run_control = 1;
}

int InterruptInit(void)
{
    const struct device *const counter_dev = DEVICE_DT_GET(COUNTER);
	int err;

	if (!device_is_ready(counter_dev)) {
		printk("counter2 device not ready.\n");
		return -1;
	}

	counter_start(counter_dev);

    alarm_cfg.flags = 0;
	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, PERIOD);
	alarm_cfg.callback = HighFreqTask;
	alarm_cfg.user_data = (void*)&alarm_cfg;

    err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID, &alarm_cfg);

	printk("Set alarm in %u sec (%u ticks)\n",
        (uint32_t)(counter_ticks_to_us(counter_dev, alarm_cfg.ticks) / USEC_PER_SEC), alarm_cfg.ticks);

	if (-EINVAL == err) {
		printk("Alarm settings invalid\n");
	} else if (-ENOTSUP == err) {
		printk("Alarm setting request not supported\n");
	} else if (err != 0) {
		printk("Error\n");
	}

    return 1;
}

int MotorControlInit(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    pwm0_ptr = new(ImplPwm);
    pwm0_ptr->Init(0);
    pwm1_ptr = new(ImplPwm);
    pwm1_ptr->Init(1);
    pwm2_ptr = new(ImplPwm);
    pwm2_ptr->Init(2);

    encoder_ptr = new(AbsoluteAngleEncoder);
    encoder_ptr->Init();

    pwm0_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_u_);
    pwm1_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_v_);
    pwm2_ptr->pwm_input_port_.ConnectTo(&pm3505_bldc.pwm_phase_w_);
    pm3505_bldc.normalize_angle_measure_.ConnectTo(&encoder_ptr->normalize_angle_measure_);
    pm3505_bldc.velocity_measure_.ConnectTo(&encoder_ptr->velocity_measure_);
    pm3505_bldc.sensor_update_time_.ConnectTo(&encoder_ptr->sensor_update_time_);

    pm3505_bldc.SetControlType(MOTOR_CONTROL_TYPE_SPEES);
    //pm3505_bldc.SetTorque(0.5);
    pm3505_bldc.SetVelocity(30);

    printk("init finished\n");

    return 0;
}

int MotorControlDeInit(void)
{
    BldcMotor& pm3505_bldc = pm3505.Create();

    pwm0_ptr->pwm_input_port_.DisConnect();
    pwm1_ptr->pwm_input_port_.DisConnect();
    pwm2_ptr->pwm_input_port_.DisConnect();
    pm3505_bldc.normalize_angle_measure_.DisConnect();
    pm3505_bldc.velocity_measure_.DisConnect();

    delete(pwm0_ptr);
    delete(pwm1_ptr);
    delete(pwm2_ptr);

    printk("deinit finished\n");

    return 0;
}
