#include <foc_controller.hpp>
#include <algorithm_utils.hpp>

#include <arm_math.h>

LOG_MODULE_REGISTER(FOC, 3);

/**
 * @brief set phase current in current loop
 *
 * @param current value for 3 phase
 * @param num numer of phase
 * @return 0 if success and -1 failed
 */
int FieldOrientedController::SetPhaseCurrent(float* current, int num)
{
    if (num < 0 || num >3 || nullptr == current) {
        return -1;
    }

    for (int idx = 0; idx < num; idx++) {
        (*currents_measured_)[idx] = current[idx];
    }

    return 0;
}

/**
 * @brief  This function transforms stator values a and b (which are
 *         directed along axes each displaced by 120 degrees) into values
 *         alpha and beta in a stationary qd reference frame.
 *                               alpha = a
 *                       beta = -(2*b+a)/sqrt(3)
 * @retval Stator values alpha and beta in alphabeta_t format
 */
void FieldOrientedController::FocClark(void)
{
    // Clarke transform
    if (currents_measured_.has_value()) {
        if (2 == current_phase_num_) {
            i_alpha_beta_measured_ = {
                (*currents_measured_)[0],
                kDivSqrt3_ * ((*currents_measured_)[1] - (*currents_measured_)[2])
            };
        } else if (3 == current_phase_num_) {
            i_alpha_beta_measured_ = {
                (*currents_measured_)[0],
                kDivSqrt3_ * (-2 * (*currents_measured_)[1] - (*currents_measured_)[0])
            };
        }
    }
}

/**
 * @brief  This function transforms stator values alpha and beta, which
 *         belong to a stationary qd reference frame, to a rotor flux
 *         synchronous reference frame (properly oriented), so as q and d.
 *                   d= alpha *cos(theta) + beta *sin(theta)
 *                   q= -alpha *sin(theta) + beta *cos(theta)
 * @param  theta: rotating frame angular position
 * @retval Stator values q and d in qd_t format
 */
void FieldOrientedController::FocPark(void)
{
    float theta = phase_measure_ + phase_velocity_measure_ *  (control_time_ - sensor_update_time_);

    if (i_alpha_beta_measured_.has_value()) {
        auto [i_alpha, i_beta] = *i_alpha_beta_measured_;
        float cos_theta = arm_cos_f32(theta);
        float sin_theta = arm_sin_f32(theta);
        i_dq_measured_ = {
            i_alpha * cos_theta + i_beta * sin_theta,
            -i_alpha * sin_theta + i_beta * cos_theta,
        };
    }
}

/**
 * @brief  This function transforms stator voltage qVq and qVd, that belong to
 *         a rotor flux synchronous rotating frame, to a stationary reference
 *         frame, so as to obtain qValpha and qVbeta:
 *                  Valfa= Vd*Cos(theta) - Vq*Sin(theta)
 *                  Vbeta= Vd*Sin(theta) + Vq*Cos(theta)
 * @param  theta: rotating frame angular position
 * @retval Stator voltage Valpha and Vbeta in qd_t format
 */
void FieldOrientedController::FocRevPark(std::optional<float2D> v_dq)
{
    float theta = phase_measure_;// + phase_velocity_measure_ *  (control_time_ - sensor_update_time_);

    auto [mod_d, mod_q] = *v_dq;

    float cos_theta = arm_cos_f32(theta);
    float sin_theta = arm_sin_f32(theta);
    v_alpha_beta_target_ = {
        mod_d * cos_theta - mod_q * sin_theta,
        mod_d * sin_theta + mod_q * cos_theta
    };
}

/**
 * @brief Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
 *        as per the magnitude invariant clarke transform
 *        The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
 * @retval 3 phase pwm and returns true on success, and false if the input was out of range
 */
std::tuple<float, float, float, bool> FieldOrientedController::FocSVM(void)
{
    float tA, tB, tC;
    int Sextant;

    if (!v_alpha_beta_target_.has_value() || __builtin_isnan(v_alpha_beta_target_->first) \
        || __builtin_isnan(v_alpha_beta_target_->second)) {
        return {0, 0, 0, false};
    }

    float alpha = v_alpha_beta_target_->first;
    float beta = v_alpha_beta_target_->second;

    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            //quadrant I
            if (kDivSqrt3_ * beta > alpha)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2
        } else {
            //quadrant II
            if (-kDivSqrt3_ * beta > alpha)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-kDivSqrt3_ * beta > alpha)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (kDivSqrt3_ * beta > alpha)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            float t1 = alpha - kDivSqrt3_ * beta;
            float t2 = kDivTwoSqrt3_ * beta;

            // PWM timings
            tA = (1.0f - t1 - t2) * 0.5f;
            tB = tA + t1;
            tC = tB + t2;
        } break;

        // sextant v2-v3
        case 2: {
            // Vector on-times
            float t2 = alpha + kDivSqrt3_ * beta;
            float t3 = -alpha + kDivSqrt3_ * beta;

            // PWM timings
            tB = (1.0f - t2 - t3) * 0.5f;
            tA = tB + t3;
            tC = tA + t2;
        } break;

        // sextant v3-v4
        case 3: {
            // Vector on-times
            float t3 = kDivTwoSqrt3_ * beta;
            float t4 = -alpha - kDivSqrt3_ * beta;

            // PWM timings
            tB = (1.0f - t3 - t4) * 0.5f;
            tC = tB + t3;
            tA = tC + t4;
        } break;

        // sextant v4-v5
        case 4: {
            // Vector on-times
            float t4 = -alpha + kDivSqrt3_ * beta;
            float t5 = -kDivTwoSqrt3_ * beta;

            // PWM timings
            tC = (1.0f - t4 - t5) * 0.5f;
            tB = tC + t5;
            tA = tB + t4;
        } break;

        // sextant v5-v6
        case 5: {
            // Vector on-times
            float t5 = -alpha - kDivSqrt3_ * beta;
            float t6 = alpha - kDivSqrt3_ * beta;

            // PWM timings
            tC = (1.0f - t5 - t6) * 0.5f;
            tA = tC + t5;
            tB = tA + t6;
        } break;

        // sextant v6-v1
        case 6: {
            // Vector on-times
            float t6 = -kDivTwoSqrt3_ * beta;
            float t1 = alpha + kDivSqrt3_ * beta;

            // PWM timings
            tA = (1.0f - t6 - t1) * 0.5f;
            tC = tA + t1;
            tB = tC + t6;
        } break;
    }

    bool result_valid =
            tA >= 0.0f && tA <= 1.0f
         && tB >= 0.0f && tB <= 1.0f
         && tC >= 0.0f && tC <= 1.0f;
    return {tA, tB, tC, result_valid};
}

/**
 * @brief current controller update
 *
 */
std::tuple<float, float, float, bool> FieldOrientedController::Update(void)
{
    std::optional<float2D> v_dq;
    float mod_to_v = (2.0f / 3.0f) * vbus_measure_;
    float v_to_mod = 1.0f / mod_to_v;

    float mod_d, mod_q;

    control_time_ = time();

    if (IsEnableCurrentControl()) {
        FocClark(); // get i_alpha_beta_measured_
        FocPark(); // get i_dq_measured_
    }

    auto [id_target, iq_target] = *i_dq_target_;
    auto [id_measure, iq_measure] = *i_dq_measured_;

    float i_err_d = id_target - id_measure;
    float i_err_q = iq_target - iq_measure;

    auto [v_d, v_q] = *v_dq_target_;

    if (IsEnableCurrentControl()) {
        mod_d = v_to_mod * (v_d + current_d_axis_pid_controller_.PIController(i_err_d));
        mod_q = v_to_mod * (v_q + current_q_axis_pid_controller_.PIController(i_err_q));

        float mod_scalefactor = 0.80f * kSqrt3Div2_ * 1.0f / std::sqrt(mod_d * mod_d + mod_q * mod_q);

        if (mod_scalefactor < 1.0f) {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
        }
    } else {
        mod_d = v_to_mod * v_d;
        mod_q = v_to_mod * v_q;
    }

    v_dq = {
        mod_d,
        mod_q
    };

    FocRevPark(v_dq);

    return FocSVM();
}
