#include <open_loop_controller.hpp>

/**
 * @brief  This function is open loop foc control task
 * @param  timestamp current time
 * @retval None
 */
void OpenLoopController::Update(uint64_t timestamp)
{
    auto [prev_id, prev_iq] = i_dq_target_.GetPrevious().value_or(float2D{0.0f, 0.0f});
    auto [prev_vd, prev_vq] = v_dq_target_.GetPrevious().value_or(float2D{0.0f, 0.0f});
    float phase = phase_target_.GetPrevious().value_or(initial_phase_);
    float phase_vel = phase_velocity_target_.GetPrevious().value_or(0.0f);

    (void)prev_iq; // unused
    (void)prev_vq; // unused

    float dt = (float)(timestamp - time_stamp_) / (float)timer_freq_;

    i_dq_target_ = {
        std::clamp(current_target_, prev_id - max_current_ramp_ * dt, prev_id + max_current_ramp_ * dt),
        0.0f
    };
    v_dq_target_ = {
        std::clamp(voltage_target_, prev_vd - max_voltage_ramp_ * dt, prev_vd + max_voltage_ramp_ * dt),
        0.0f
    };

    phase_vel = std::clamp(velocity_target_, phase_vel - max_phase_vel_ramp_ * dt, phase_vel + max_phase_vel_ramp_ * dt);
    phase_velocity_target_ = phase_vel;

    phase_target_ = WrapPmPi(phase + phase_vel * dt);
    total_distance_ = total_distance_.GetPrevious().value_or(0.0f) + phase_vel * dt;

    time_stamp_ = timestamp;
}
