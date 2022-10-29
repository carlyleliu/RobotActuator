#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>

#include <motor_abstract.hpp>
#include <bldc_motor.hpp>
#include <angle_encoder_abstract.hpp>
#include <absolute_encoder.hpp>
#include <motor_control_factory.hpp>
#include <interrupt.hpp>

#include <impl_pwm.hpp>
#include <impl_adc.hpp>

LOG_MODULE_REGISTER(Main, 7);

int8_t run_control = 0;

int main(void)
{
    uint32_t start_ticks = 0, stop_ticks = 0, nanoseconds_spent = 0;
	printk("start main\n");

    MotorControlInit();
    //InterruptInit();


    while (1) {
        //if (1 == run_control) {
            start_ticks = k_cycle_get_32();
            MotorControlUpdate();
            stop_ticks = k_cycle_get_32();

            nanoseconds_spent = (uint32_t)k_cyc_to_ns_floor64(stop_ticks - start_ticks);

            printk("delta = %d\n", nanoseconds_spent / 1000);
            run_control = 0;
        //}
        //k_busy_wait(200);
        //printk("run_control = %d\n", run_control);
	}

    return 1;
}
