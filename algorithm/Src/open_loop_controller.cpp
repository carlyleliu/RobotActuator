#include <open_loop_controller.hpp>

LOG_MODULE_REGISTER(OpenLoopController, 3);

/**
 * @brief  This function is open loop foc control task
 * @param  timestamp current time
 * @retval None
 */
void OpenLoopController::Update(void)
{
    auto [prev_id, prev_iq] = *i_dq_target_;
    auto [prev_vd, prev_vq] = *v_dq_target_;

    (void)prev_iq; // unused
    (void)prev_vd; // unused

    float dt = 0.001;

    i_dq_target_ = {
        std::clamp(current_target_, prev_id - max_current_ramp_ * dt, prev_id + max_current_ramp_ * dt),
        0.0f
    };
    v_dq_target_ = {
        std::clamp(voltage_target_, prev_vd - max_voltage_ramp_ * dt, prev_vd + max_voltage_ramp_ * dt),
        0.0f
    };

    if (!align_mode_) {
        phase_velocity_target_ = std::clamp(velocity_target_, phase_velocity_target_ - max_phase_vel_ramp_ * dt, \
                                            phase_velocity_target_ + max_phase_vel_ramp_ * dt);

        phase_target_ = WrapPmPi(phase_target_ + phase_velocity_target_ * dt);
    } else {
        phase_velocity_target_ = 0;
        phase_target_ = 0;
    }

    LOG_DBG("iqd[%f %f] vqd[%f %f] phase_target_[%f] phase_vel[%f]\n", \
            prev_id, prev_iq, prev_vd, prev_vq, phase_target_, phase_velocity_target_);
}

/**
 * @brief  This function for test
 * @param  None
 * @retval None
 */
void OpenLoopController::Test(void)
{
    float update_time = time();

    SetMaxCurrentRamp(2);
    SetMaxVoltageRamp(5);
    SetMaxPhaseVelRamp(5);

    SetVelocityTarget(100);
    SetCurrentTarget(0);
    SetVoltageTarget(4);
    SetInitialPhase(0);

    Update();

    if (nullptr != foc_ptr_) {
        foc_ptr_->SetPhaseAngle(phase_target_);
        foc_ptr_->SetPhaseVelocity(phase_velocity_target_);
        foc_ptr_->SetSensorUpdateTime(update_time);
        foc_ptr_->SetVbus(12);
        foc_ptr_->SetVdq(v_dq_target_);
        //foc_ptr_->SetIdq(i_dq_target_);
    } else {
        LOG_ERR("foc_ptr is nullptr\n");
    }

}
