#include <absolute_encoder.hpp>

LOG_MODULE_DECLARE(EncoderSensor, LOG_LEVEL_WRN);

/**
  * @brief absolute angle encoder Sensor ImplInit
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplInit(void)
{
    dev_ = DEVICE_DT_GET_ONE(infineon_gmr);

    if (!device_is_ready(dev_)) {
		LOG_ERR("Device get binding device\n");
        return -EIO;
	}

    return 0;
}

/**
  * @brief absolute angle encoder Sensor ImplDeInit
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplDeInit(void)
{
    dev_ = NULL;

    return 0;
}

/**
  * @brief absolute angle encoder Sensor ImplCalibration
  * @param None
  * @retval None
  */
int AbsoluteAngleEncoder::ImplCalibration(void)
{

    return 0;
}

/**
  * @brief get true angle from sensor
  * @param None
  * @retval None
  */
uint16_t AbsoluteAngleEncoder::ImplGetAbsoluteAngle(void)
{
    sensor_sample_fetch(dev_);

    sensor_channel_get(dev_, SENSOR_CHAN_ALL, &angle_);

    return (uint16_t)angle_.val1;
}


