#include <led_device.hpp>

/* test times */
#define TEST_TIMES      1000

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/* The devicetree node identifier for the "led1" alias. */
#define LED1_NODE DT_ALIAS(led1)

constexpr uint8_t kLedDeviceNum = 2;

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static struct gpio_dt_spec kGpioSpecArray[kLedDeviceNum] = {
    GPIO_DT_SPEC_GET(LED0_NODE, gpios),
    GPIO_DT_SPEC_GET(LED1_NODE, gpios)
};

/**
 * @brief LedInit
 * @param None
 * @return None
 */
int LedInit(void)
{
    int ret = 0;
    struct gpio_dt_spec* spec = NULL;

    for (int idx = 0; idx < kLedDeviceNum; idx++) {
        spec = &kGpioSpecArray[idx];

        ret = device_is_ready(spec->port);
        if (!ret) {
            return ret;
        }

        ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT_ACTIVE);
        if (!ret) {
            return ret;
        }
    }
}

/**
 * @brief Led ON
 * @param None
 * @return status
 */
int LedOn(uint8_t id)
{
    int ret = 0;
    struct gpio_dt_spec* spec = NULL;

    if (id >= kLedDeviceNum) {
        return -1;
    }

    spec = &kGpioSpecArray[id];
    ret = gpio_pin_set_dt(spec, 1);

    return ret;
}

/**
 * @brief Led OFF
 * @param None
 * @return status
 */
int LedOff(uint8_t id)
{
    int ret = 0;
    struct gpio_dt_spec* spec = NULL;

    if (id >= kLedDeviceNum) {
        return -1;
    }

    spec = &kGpioSpecArray[id];
    ret = gpio_pin_set_dt(spec, 0);

    return ret;
}

/**
 * @brief Led Toggle
 * @param None
 * @return status
 */
int LedToggle(uint8_t id)
{
    int ret = 0;
    struct gpio_dt_spec* spec = NULL;

    if (id >= kLedDeviceNum) {
        return -1;
    }

    spec = &kGpioSpecArray[id];
    ret = gpio_pin_toggle_dt(spec);

    return ret;
}

/**
 * @brief LedBlinkTest
 * @param None
 * @return None
 */
int LedTest(void)
{
    int ret;
    struct gpio_dt_spec* spec = NULL;

    for (int i = 0; i < TEST_TIMES; i++) {
        for (int idx = 0; idx < kLedDeviceNum; idx++) {
            spec = &kGpioSpecArray[idx];

            ret = gpio_pin_toggle_dt(spec);
            if (ret < 0) {
            	return ret;
            }
        }
	    k_msleep(SLEEP_TIME_MS);
    }

    return ret;
}
