#ifndef __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__
#define __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__

#include <component_port.hpp>

#include <cmath>
#include <utility>

constexpr float M_PI = 3.14159265358979323846f;
using float2D = std::pair<float, float>;

class OpenLoopController
{
  public:
    OpenLoopController();
    ~OpenLoopController();

    void Update(uint64_t timestamp);

  private:
    float WrapPmPi(float x)
    {
        float intval = nearbyintf(x / 2 * M_PI);
        return x - intval * 2 * M_PI;
    };

  private:
    // Config
    float max_current_ramp_;    // [A/s]
    float max_voltage_ramp_;    // [V/s]
    float max_phase_vel_ramp_;  // [rad/s^2]

    // Inputs
    float velocity_target_;
    float current_target_;
    float voltage_target_;
    float initial_phase_;

    // State/Outputs
    uint32_t timer_freq_;
    uint64_t time_stamp_;

    OutputPort<float2D> i_dq_target_;
    OutputPort<float2D> v_dq_target_;
    OutputPort<float> phase_target_;
    OutputPort<float> phase_velocity_target_;
    OutputPort<float> total_distance_;
};

#endif // ! __MIDDLEWARE_MOTOR_CONTROL_OPEN_LOOP_CONTROLLER_HPP__
