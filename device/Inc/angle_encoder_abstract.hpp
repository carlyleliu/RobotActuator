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
        angle_measure_prev_(0),
        angle_measure_(0),
        phase_measure_(0),
        angle_offset_(1300), //1180
        position_measure_(0),
        normalized_angle_measure_(0),
        total_angle_measure_(0),
        velocity_(0.0f),
        rotate_direction_(1),
        mechanical_to_phase_direction_(-1),
        pole_pairs_(11),
        circle_counter_(0),
        calibrationed_(0),
        inited_(0),
        aligned_(0),
        measure_normalize_angle_(0.0f),
        measure_velocity_(0.0f)
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
    uint16_t GetOriginAngle(void) {
        return ImplGetAbsoluteAngle();
    };
    float GetNormalizeAngle(void) {
        return normalized_angle_measure_;
    };
    float GetVelocity(void) {
        return velocity_;
    };
    float GetTime(void) {
        return time_;
    };

    void SetPolePairs(uint8_t num) {
        pole_pairs_ = num;
    };

  protected:
    float time_;

    int16_t angle_measure_prev_;
    int16_t angle_measure_;
    int16_t phase_measure_;

    int16_t angle_offset_;

    float position_measure_;

    float normalized_angle_measure_;
    float total_angle_measure_;

    float velocity_;

    int8_t rotate_direction_;
    int8_t mechanical_to_phase_direction_;

    uint8_t pole_pairs_;
    uint32_t circle_counter_;

    uint8_t calibrationed_ : 1;
    uint8_t inited_ : 1;
    uint8_t aligned_ : 1;
  public:
    OutputPort<float> measure_normalize_angle_;
    OutputPort<float> measure_velocity_;
};

#endif  // ! __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
