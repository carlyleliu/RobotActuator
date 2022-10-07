#ifndef __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
#define __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__

#include <zephyr/kernel.h>
#include <cmath>
#include <algorithm_utils.hpp>
#include <algorithm>

class AngleEncoderAbstract {
  public:
    AngleEncoderAbstract() :
        direction_(1){};
    ~AngleEncoderAbstract(){};
    virtual void Init(void) = 0;
    virtual void DeInit(void) = 0;
    virtual uint16_t GetAbsoluteAngle(void) = 0;
    virtual void UpdateData(void) = 0;
    virtual void Notify(void) = 0;

  protected:
    volatile uint64_t time_stamp_;

    int16_t mechanical_angle_measure_prev_;
    int16_t mechanical_angle_measure_;
    int16_t electronic_angle_measure_;

    int16_t mechanical_angle_offset_;
    int16_t electronic_angle_offset_;

    float mechanical_speed_measure_;
    float electronic_speed_measure_;

    float mechanical_position_measure_;
    float electronic_position_measure_;

    float normalized_angle_measure_;
    float total_angle_measure_;

    float rpm_;
    float last_rpm_;

    int8_t direction_;

    uint8_t number_of_pole_pairs_;
    uint32_t circle_counter_;

    uint8_t calibrationed_ : 1;
    uint8_t inited_ : 1;
    uint8_t aligned_ : 1;
};

#endif  // ! __MIDDLEWARE_SENSOR_ANGLE_ENCODER_ABSTRACT_HPP__
