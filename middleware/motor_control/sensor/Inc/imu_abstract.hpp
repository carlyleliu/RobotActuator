#ifndef __MIDDLEWARE_SENSOR_IMU_ABSTRACT_HPP__
#define __MIDDLEWARE_SENSOR_IMU_ABSTRACT_HPP__

class ImuAbstract
{
  public:
    ImuAbstract();
    ~ImuAbstract();
    virtual void Init() = 0;
    virtual void DeInit() = 0;
    virtual void UpdateData() = 0;
    virtual void Notify() = 0;
  protected:
    float accel_normalized_[3];          /* 3 axis acc normalized data */
    float gyro_normalized_[3];           /* 3 axis gyro normalized data */
    float magnetometer_normalized_[3];   /* 3 axis magnetometer normalized data */
    float accel_scale_;                  /* accel scale data */
    float gyro_scale_;                   /* gyro scale data */
    float magnetometer_scale_;           /* magnetometer scale data */
    float accel_cal_offset_[3];          /* accel calibration data. static offset correction */
    float accel_cal_scale_[3];           /* accel calibration data. scale error correction */
    float gyro_cal_offset_[3];           /* gyro calibration data. static offset correct */
    float temperature_;                  /* sensor temperature value */
    volatile uint64_t time_stamp_;       /* this data time stamp */
    uint8_t gyro_has_calibration_ : 1;   /* flag gyro has calibration */
    uint8_t accel_has_calibration_ : 1;  /* flag accel has calibration */
    uint8_t accel_err_ : 1;              /* flag of accel read error */
    uint8_t gyro_err_ : 1;               /* flag of gyro read error */
    uint8_t inited_ : 1;
};

#endif // ! __MIDDLEWARE_SENSOR_IMU_ABSTRACT_HPP__
