#ifndef __MIDDLEWARE_ALGORITHM_FOC_CONTROLLER_HPP__
#define __MIDDLEWARE_ALGORITHM_FOC_CONTROLLER_HPP__

#include <component_port.hpp>

#include <optional>
#include <string>
#include <vector>
#include <array>
#include <tuple>

#define PHASE_NUM   3

#define SECTOR_1    0u
#define SECTOR_2    1u
#define SECTOR_3    2u
#define SECTOR_4    3u
#define SECTOR_5    4u
#define SECTOR_6    5u
#define SQRT3FACTOR (uint16_t)0xDDB4 /* = (16384 * 1.732051 * 2)*/

using float2D = std::pair<float, float>;

// Math Constants
constexpr uint8_t kPhaseNum = 3;

class FieldOrientedController
{
  public:
    FieldOrientedController():
    current_phase_num_(kPhaseNum) {};
    ~FieldOrientedController() {};
    void FocClark(void);
    void FocPark(void);
    void FocRevPark(void);
    std::tuple<float, float, float, bool> FocSVM(void);

  private:
    const uint8_t current_phase_num_;
    float electrical_angle_;
    float vbus_voltage_measured_;

    InputPort<float2D> i_dq_target_external_;
    InputPort<float2D> v_dq_target_external_;
    InputPort<float> phase_target_external_;
    InputPort<float> phase_velocity_target_external_;

    std::optional<float2D> i_alpha_beta_measured_;
    std::optional<float2D> i_alpha_beta_target_;
    std::optional<float2D> v_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_target_;
    std::optional<float2D> i_dq_measured_;
    std::optional<float2D> i_dq_target_;
    std::optional<float2D> v_dq_measured_;
    std::optional<float2D> v_dq_target_;
    std::optional<std::array<float, kPhaseNum>> currents_measured_;

    uint64_t timestamp_;
};

#endif // ! __MIDDLEWARE_ALGORITHM_FOC_CONTROLLER_HPP__

