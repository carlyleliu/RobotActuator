#ifndef __ALGORITHM_FOC_CONTROLLER_HPP__
#define __ALGORITHM_FOC_CONTROLLER_HPP__

#include <component_port.hpp>
#include <pid_controller.hpp>

#include <time_util.h>

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

using float2D = std::pair<float, float>;

// Math Constants
constexpr uint8_t kPhaseNum = 3;

class FieldOrientedController
{
  public:
    FieldOrientedController():
    current_phase_num_(kPhaseNum),
    enable_current_control_(false),
    sensor_update_time_(0.0f),
    control_time_(0.0f),
    phase_measure_(0.0f),
    phase_velocity_measure_(0.0f),
    vbus_measure_(12)
    {};
    ~FieldOrientedController() {};
    void FocClark(void);
    void FocPark(void);
    void FocRevPark(std::optional<float2D> v_dq);
    std::tuple<float, float, float, bool> FocSVM(void);
    std::tuple<float, float, float, bool> Update(void);

    /* current loop */
    void EnableCurrentControl(void) {
        enable_current_control_ = true;
    };
    void DisableCurrentControl(void) {
        enable_current_control_ = false;
    };
    bool IsEnableCurrentControl(void) {
        return enable_current_control_;
    };

    /* update phase current */
    int SetPhaseCurrent(float* current, int num);

    /* update phase angle */
    void SetPhaseAngle(float phase) {
        phase_measure_ = phase;
    };
    void SetPhaseVelocity(float velocity) {
        phase_velocity_measure_ = velocity;
    };
    void SetSensorUpdateTime(float time) {
        sensor_update_time_ = time;
    };

    /* update i_dq and v_dq */
    void SetIdq(std::optional<float2D> i_dq) {
        i_dq_target_ = i_dq;
    };
    void SetVdq(std::optional<float2D> v_dq) {
        v_dq_target_ = v_dq;
    };

    /* update vbus */
    void SetVbus(float vbus) {
        vbus_measure_ = vbus;
    };

    PidController& GetDAxisPidHandler(void) {
        return current_d_axis_pid_controller_;
    };
    PidController& GetQAxisPidHandler(void) {
        return current_q_axis_pid_controller_;
    };


  private:
    const uint8_t current_phase_num_;
    bool enable_current_control_;

    /* s */
    float sensor_update_time_;
    float control_time_;

    /* phase angle */
    float phase_measure_;
    float phase_velocity_measure_;

    float vbus_measure_;

    std::optional<float2D> i_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_measured_;
    std::optional<float2D> v_alpha_beta_target_;
    std::optional<float2D> i_dq_measured_;
    std::optional<float2D> i_dq_target_;
    std::optional<float2D> v_dq_target_;
    std::optional<std::array<float, kPhaseNum>> currents_measured_;

    PidController current_d_axis_pid_controller_;
    PidController current_q_axis_pid_controller_;
};

#endif // ! __ALGORITHM_FOC_CONTROLLER_HPP__

