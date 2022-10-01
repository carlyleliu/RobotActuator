#ifndef __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__

#include <component_port.hpp>
#include <algorithm_macro.hpp>

#include <cmath>
#include <utility>

#include <zephyr/logging/log.h>

using float2D = std::pair<float, float>;

class OpenLoopController
{
  public:
    OpenLoopController() :
    timer_freq_(170000000),
    time_stamp_(0),
    i_dq_target_(std::make_pair(0.0,0.0)),
    v_dq_target_(std::make_pair(0.0,0.0)),
    phase_target_(0.0),
    phase_velocity_target_(0.0){};
    ~OpenLoopController();

    /* set OpenLoopController config param */
    void SetMaxCurrentRamp(float max_current) { max_current_ramp_ = max_current; };
    void SetMaxVoltageRamp(float max_voltage) { max_voltage_ramp_ = max_voltage; };
    void SetMaxPhaseVelRamp(float max_phase_vel) { max_phase_vel_ramp_ = max_phase_vel; };

    void SetVelocityTarget(float vel) { velocity_target_ = vel; };
    void SetCurrentTarget(float current) { current_target_ = current; };
    void SetVoltageTarget(float voltage) { voltage_target_ = voltage; };
    void SetInitialPhase(float phase) { initial_phase_ = phase; };

    void SetTimerFreq(uint32_t freq) { timer_freq_ = freq; };

    /* get OpenLoopController config param */
    float GetMaxCurrentRamp(void) { return max_current_ramp_; };
    float GetMaxVoltageRamp(void) { return max_voltage_ramp_; };
    float GetMaxPhaseVelRamp(void) { return max_phase_vel_ramp_; };
    float GetVelocityTarget(void) { return velocity_target_; };
    float GetCurrentTarget(void) { return current_target_; };
    float GetVoltageTarget(void) { return voltage_target_; };
    float GetInitialPhase(void) { return initial_phase_; };
    float GetTimerFreq(void) { return timer_freq_; };

    /* export function interface */
    void Update(uint64_t timestamp);
    void Reset(void);
    void Test(void);

  private:
    float WrapPmPi(float x)
    {
        float intval = nearbyintf(x / (2 * kPI_));
        return x - intval * 2 * kPI_;
    };

  private:
    // Config
    float max_current_ramp_;    // [A/s]
    float max_voltage_ramp_;    // [V/s]
    float max_phase_vel_ramp_;  // [rad/s]

    // Inputs
    float velocity_target_;
    float current_target_;
    float voltage_target_;

    float initial_phase_;

    // State/Outputs
    uint32_t timer_freq_;
    uint64_t time_stamp_;

  public:
    OutputPort<float2D> i_dq_target_;
    OutputPort<float2D> v_dq_target_;

    OutputPort<float> phase_target_;
    OutputPort<float> phase_velocity_target_;
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__
