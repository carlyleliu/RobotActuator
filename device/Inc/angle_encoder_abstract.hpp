#ifndef __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
#define __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <time_util.h>

#include <cmath>
#include <algorithm_utils.hpp>
#include <algorithm>
#include <component_port.hpp>

class AngleEncoderAbstract {
  public:
    AngleEncoderAbstract() :
        time_(0.0f),
        mechanical_angle_measure_prev_(0),
        mechanical_angle_measure_(0),
        electronic_angle_measure_(0),
        mechanical_angle_offset_(1300), //1180
        mechanical_position_measure_(0),
        normalized_angle_measure_(0),
        total_angle_measure_(0),
        rpm_(0.0f),
        rotate_direction_(1),
        mechanical_to_phase_direction_(-1),
        number_of_pole_pairs_(11),
        circle_counter_(0),
        calibrationed_(0),
        inited_(0),
        aligned_(0),
        measure_normalize_angle_(0.0f),
        measure_rpm_(0.0f)
        {};
    virtual ~AngleEncoderAbstract() {};
    virtual int ImplInit(void) = 0;
    virtual int ImplDeInit(void) = 0;
    virtual int ImplCalibration(void) = 0;
    virtual uint16_t ImplGetAbsoluteAngle(void) = 0;

    int Init(void);
    int DeInit(void);
    int Align(void);
    int Calibration(void);
    int Update(void);
    int Notify(void);
    uint16_t GetOriginAngle(void) { return ImplGetAbsoluteAngle(); };
    float GetNormalizeAngle(void) { return normalized_angle_measure_; };
    float GetRPM(void) { return rpm_; };
    float GetTime(void) { return time_; };

    void SetNumPolePairs(uint8_t num) { number_of_pole_pairs_ = num; };

  protected:
    float time_;

    int16_t mechanical_angle_measure_prev_;
    int16_t mechanical_angle_measure_;
    int16_t electronic_angle_measure_;

    int16_t mechanical_angle_offset_;

    float mechanical_position_measure_;

    float normalized_angle_measure_;
    float total_angle_measure_;

    float rpm_;

    int8_t rotate_direction_;
    int8_t mechanical_to_phase_direction_;

    uint8_t number_of_pole_pairs_;
    uint32_t circle_counter_;

    uint8_t calibrationed_ : 1;
    uint8_t inited_ : 1;
    uint8_t aligned_ : 1;
  public:
    OutputPort<float> measure_normalize_angle_;
    OutputPort<float> measure_rpm_;
};

#endif  // ! __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
