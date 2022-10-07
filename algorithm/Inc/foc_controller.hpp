#ifndef __ALGORITHM_FOC_CONTROLLER_HPP__
#define __ALGORITHM_FOC_CONTROLLER_HPP__

#include <component_port.hpp>
#include <pid_controller.hpp>

#include <optional>
#include <string>
#include <vector>
#include <array>
#include <tuple>

#include <sys/time.h>
#include <zephyr/logging/log.h>

#define PHASE_NUM   3

#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u
#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

#define US_TO_S     (1000*1000)

using float2D = std::pair<float, float>;

// Math Constants
constexpr uint8_t kPhaseNum = 3;

class FieldOrientedController
{
  public:
    FieldOrientedController():
    current_phase_num_(kPhaseNum),
    vbus_max_(12),
    vbus_measured_(12),
    enable_current_control_(false),
    pwm_phase_u_(0.0),
    pwm_phase_v_(0.0),
    pwm_phase_w_(0.0) {};
    ~FieldOrientedController() {};
    void FocClark(void);
    void FocPark(void);
    void FocRevPark(std::optional<float2D> v_dq);
    std::tuple<float, float, float, bool> FocSVM(void);

    void Update(void);

    int SetPhaseCurrent(float* current, int num);

    void SetVBusMax(float vbus) { vbus_max_ = vbus; };
    void SetVBusMeasured(float vbus) { vbus_measured_ = vbus; };
    void EnableCurrentControl(void) { enable_current_control_ = true; };
    void DisableCurrentControl(void) { enable_current_control_ = false; };

    float GetVBusMax(void) { return vbus_max_; };
    float GetVBusMeasured(void) { return vbus_measured_; };
    bool IsEnableCurrentControl(void) { return enable_current_control_; };

  private:
    const uint8_t current_phase_num_;
    float vbus_max_;
    float vbus_measured_;
    bool enable_current_control_;

    std::optional<float2D> i_alpha_beta_measured_;
    //std::optional<float2D> i_alpha_beta_target_;
    std::optional<float2D> v_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_target_;
    std::optional<float2D> i_dq_measured_;
    std::optional<float2D> i_dq_target_;
    std::optional<float2D> v_dq_measured_;
    std::optional<float2D> v_dq_target_;
    std::optional<std::array<float, kPhaseNum>> currents_measured_;

    /* us */
    uint64_t sensor_update_timestamp_;
    uint64_t control_timestamp_;

    PidController current_d_axis_pid_controller_;
    PidController current_q_axis_pid_controller_;

  public:
    InputPort<float2D> i_dq_target_external_;
    InputPort<float2D> v_dq_target_external_;
    InputPort<float> phase_target_external_;
    InputPort<float> phase_velocity_target_external_;

    OutputPort<float> pwm_phase_u_;
    OutputPort<float> pwm_phase_v_;
    OutputPort<float> pwm_phase_w_;
};

#endif // ! __ALGORITHM_FOC_CONTROLLER_HPP__

