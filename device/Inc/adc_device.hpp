#ifndef __MIDDLEWARE_SENSOR_ADC_DEVICE_HPP__
#define __MIDDLEWARE_SENSOR_ADC_DEVICE_HPP__

#include <zephyr/kernel.h>
#include <sys/time.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

union AdcValue
{
    uint16_t adc_16bit_value_[2];
    uint32_t adc_32bit_value_;
};

/* vbus temp current */

class AdcDeviceAbstract
{
  public:
    AdcDeviceAbstract() :
        time_stamp_(0),
        adc_resolution_(12) {};
    ~AdcDeviceAbstract();

    void SetDividerRatio(float ratio) { divider_ratio_ = ratio; };
    float GetDividerRatio(void) { return divider_ratio_; };

    void SetReferenceValue(float value) { adc_reference_value_ = value; };
    float GetReferenceValue(void) { return adc_reference_value_; };

    void SetResolution(uint16_t resolution) { adc_resolution_ = resolution; };
    uint16_t GeRresolution(void) { return adc_resolution_; };

    void SetAdcChannel(uint16_t channel) { adc_channel_ = channel; }

    float GetChannelValue(void) { return value_; };

    int Update(void);
    int Init(void);

  private:
    uint64_t time_stamp_;
    uint16_t adc_channel_;
    uint16_t adc_resolution_;
    float divider_ratio_;

    float adc_reference_value_;

    AdcValue raw_value_;
    float value_;

    struct adc_dt_spec* adc_spec_;
    struct adc_sequence sequence_;
};

#endif // ! __MIDDLEWARE_SENSOR_ADC_DEVICE_HPP__

