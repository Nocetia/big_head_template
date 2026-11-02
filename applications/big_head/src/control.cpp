#include "control.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace big_head
{
namespace
{
constexpr float pi = 3.14159265358979323846f;
constexpr float control_period_s = 0.001f;
constexpr auto measured_derivative =
    control::pid_mode::position | control::pid_mode::derivative_on_measurement;

bool finite(float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

control::pid make_pid(const float values[5], control::pid_mode mode)
{
    return control::pid(values[0], values[1], values[2], values[3], values[4], mode);
}

float wrap_angle(float angle)
{
    return std::remainder(angle, 2.0f * pi);
}

float slew(float current, float target, float step)
{
    return current + std::clamp(target - current, -step, step);
}

float feedforward(float velocity, float acceleration, const float parameters[4])
{
    return (parameters[1] * acceleration + parameters[2] * velocity) * parameters[3];
}

bool finite_attitude(const control_input& input)
{
    return finite(input.pitch) && finite(input.yaw) && finite(input.total_yaw) &&
           finite(input.gyro_pitch) && finite(input.gyro_yaw);
}
} // namespace

controller::controller(const control_parameters& parameters)
    : config_(parameters),
      yaw_position_(make_pid(parameters.yaw_position_pid, measured_derivative)),
      yaw_velocity_(make_pid(parameters.yaw_velocity_pid, control::pid_mode::position)),
      pitch_position_(make_pid(parameters.pitch_position_pid, measured_derivative)),
      pitch_velocity_(make_pid(parameters.pitch_velocity_pid, measured_derivative)),
      gimbal_only_(parameters.gimbal_only_default)
{
}

float controller::scalar_filter::update(float sample, const float parameters[2])
{
    const float predicted_covariance = covariance + parameters[0];
    const float gain = predicted_covariance / (predicted_covariance + parameters[1]);
    value += gain * (sample - value);
    covariance = (1.0f - gain) * predicted_covariance;
    return value;
}

void controller::scalar_filter::reset(float sample)
{
    value = sample;
    covariance = 0.02f;
}

void controller::reset_gimbal(const control_input& input)
{
    const bool valid = finite_attitude(input);
    yaw_reference_ = valid ? input.total_yaw : 0.0f;
    pitch_reference_ = valid ? input.pitch : 0.0f;
    yaw_position_.reset_state(yaw_reference_, yaw_reference_);
    pitch_position_.reset_state(pitch_reference_, pitch_reference_);
    yaw_velocity_.reset_state(0.0f, valid ? input.gyro_yaw : 0.0f);
    pitch_velocity_.reset_state(0.0f, valid ? input.gyro_pitch : 0.0f);
    yaw_held_ = false;
    pitch_held_ = false;
    auto_fire_active_ = false;
    mode_ = gimbal_mode::relax;
}

void controller::reset_remote(const control_input& input)
{
    source_ = input.remote_valid ? input.source : remote_source::none;
    previous_remote_ = input.remote;
    armed_ = false;
    relaxed_seen_ = input.remote_valid && input.remote.left_sw == remotes::sw_state::low;
    shooter_ = shooter_mode::closed;
    high_frequency_ = true;
    aim_rune_ = false;
    gimbal_only_ = config_.gimbal_only_default;
    leg_level_ = 0;
    keyboard_vx_ = 0.0f;
    keyboard_vy_ = 0.0f;
    auto_fire_blocked_ = false;
    mouse_yaw_.reset();
    mouse_pitch_.reset();
    reset_gimbal(input);
}

control_output controller::update(const control_input& input)
{
    control_output output{};
    output.bullet_count = bullet_count_;
    if (!input.remote_valid || input.source == remote_source::none || input.source != source_)
    {
        // A takeover must not inherit the other operator's toggles or button edges.
        reset_remote(input);
        return output;
    }

    const auto& remote = input.remote;
    const bool new_frame = input.remote_new_frame;
    const bool relaxed = remote.left_sw == remotes::sw_state::low || remote.pause;
    if (relaxed)
    {
        relaxed_seen_ = true;
        armed_ = false;
    }
    else if (new_frame)
    {
        const bool switch_armed =
            relaxed_seen_ && previous_remote_.left_sw == remotes::sw_state::low;
        const bool keyboard_armed = remote.key.CTRL && remote.key.X && !previous_remote_.key.X;
        if (switch_armed || keyboard_armed)
        {
            armed_ = true;
        }
    }
    if (remote.key.CTRL && remote.key.Z)
    {
        armed_ = false;
    }

    bool turn = false;
    bool jump = false;
    bool fly = false;
    bool stair = false;
    if (armed_ && new_frame)
    {
        if (input.source == remote_source::vt03)
        {
            if (remote.fn_1 && !previous_remote_.fn_1)
            {
                shooter_ = shooter_mode::closed;
            }
            else if (remote.fn_2 && !previous_remote_.fn_2)
            {
                shooter_ = static_cast<shooter_mode>((static_cast<unsigned>(shooter_) + 1U) % 3U);
            }
        }
        if (remote.key.Q && !previous_remote_.key.Q)
        {
            leg_level_ = std::max<int8_t>(0, leg_level_ - 1);
        }
        if (remote.key.E && !previous_remote_.key.E)
        {
            leg_level_ = std::min<int8_t>(2, leg_level_ + 1);
        }
        if (remote.key.X && !previous_remote_.key.X && !remote.key.CTRL)
        {
            high_frequency_ = !high_frequency_;
        }
        if (remote.key.F && !previous_remote_.key.F)
        {
            aim_rune_ = !aim_rune_;
            if (remote.key.CTRL)
            {
                gimbal_only_ = !gimbal_only_;
            }
        }
        turn = remote.key.R && !previous_remote_.key.R;
        jump = remote.key.C && !previous_remote_.key.C;
        fly = remote.key.G && !previous_remote_.key.G;
        stair = remote.key.V && !previous_remote_.key.V;
    }
    if (input.source == remote_source::dr16)
    {
        shooter_ = static_cast<shooter_mode>(remote.right_sw);
    }
    if (new_frame)
    {
        previous_remote_ = remote;
    }

    const bool ready = armed_ && input.imu_valid && finite_attitude(input) && input.pitch_online;
    if (!ready)
    {
        shooter_ = shooter_mode::closed;
        leg_level_ = 0;
        aim_rune_ = false;
        keyboard_vx_ = 0.0f;
        keyboard_vy_ = 0.0f;
        auto_fire_blocked_ = false;
        mouse_yaw_.reset();
        mouse_pitch_.reset();
        reset_gimbal(input);
        return output;
    }

    output.armed = true;
    output.auto_aim = config_.auto_aim_default || remote.mouse_right || remote.button;
    output.auto_shoot = config_.auto_shoot_default || (output.auto_aim && remote.key.CTRL);
    output.aim_rune = aim_rune_;
    const bool spin = remote.key.SHIFT || remote.left_sw == remotes::sw_state::up;
    if (spin)
    {
        leg_level_ = 0;
    }
    const float keyboard_x = static_cast<float>(remote.key.W) - remote.key.S;
    const float keyboard_y = static_cast<float>(remote.key.D) - remote.key.A;
    keyboard_vx_ = slew(keyboard_vx_, keyboard_x, config_.keyboard_slew_per_tick);
    keyboard_vy_ = slew(keyboard_vy_, keyboard_y, config_.keyboard_slew_per_tick);
    if (input.lower_valid)
    {
        output.lower.ifmove = true;
        output.lower.ifspin = spin;
        output.lower.ifjump = jump;
        output.lower.ifturn = turn;
        output.lower.iffly = fly;
        output.lower.ifstair = stair;
        output.lower.ifgimbalonly = gimbal_only_;
        output.lower.len_level = leg_level_;
        output.lower.vx = static_cast<int8_t>((keyboard_vx_ + remote.left_y) * 10.0f);
        output.lower.vy = static_cast<int8_t>((keyboard_vy_ + remote.left_x) * 10.0f);
    }
    output.ui.reset = remote.key.B;
    output.ui.bulletfreq = high_frequency_ && !aim_rune_;
    output.ui.aim_rune = aim_rune_;

    const auto& vision = input.vision;
    const bool finite_vision = finite(vision.yaw) && finite(vision.yaw_vel) &&
                               finite(vision.yaw_acc) && finite(vision.pitch) &&
                               finite(vision.pitch_vel) && finite(vision.pitch_acc);
    const float predicted_yaw = vision.yaw + vision.yaw_vel * config_.yaw_feedforward[0];
    // Preserve the legacy +/-90 degree acquisition window, including the wrap at +/-pi.
    const bool target_accepted = input.vision_valid && finite_vision && output.auto_aim &&
                                 std::fabs(wrap_angle(predicted_yaw - input.yaw)) <= pi * 0.5f;
    const gimbal_mode requested_mode =
        target_accepted ? gimbal_mode::automatic : gimbal_mode::manual;
    const bool mode_changed = mode_ != requested_mode;
    if (mode_changed)
    {
        reset_gimbal(input);
        mode_ = requested_mode;
        vision_yaw_.reset(predicted_yaw);
        vision_pitch_.reset(vision.pitch + vision.pitch_vel * config_.pitch_feedforward[0]);
        if (mode_ == gimbal_mode::manual)
        {
            // AUTO does not consume mouse displacement. Do not replay the old
            // manual filter state when vision releases control.
            mouse_yaw_.reset();
            mouse_pitch_.reset();
        }
    }

    float yaw_speed_reference = 0.0f;
    float pitch_speed_reference = 0.0f;
    float yaw_effort_ff = 0.0f;
    float pitch_effort_ff = 0.0f;
    bool pitch_reachable = false;
    const bool yaw_ready = input.lower_valid && (input.lower.inited || gimbal_only_);
    if (mode_ == gimbal_mode::automatic)
    {
        const float unwrapped_vision =
            vision_yaw_.value + wrap_angle(predicted_yaw - vision_yaw_.value);
        const float filtered_yaw = vision_yaw_.update(unwrapped_vision, config_.vision_filter);
        yaw_reference_ = input.total_yaw + wrap_angle(filtered_yaw - input.yaw);
        const float predicted_pitch =
            vision.pitch + vision.pitch_vel * config_.pitch_feedforward[0];
        pitch_reference_ = std::clamp(-vision_pitch_.update(predicted_pitch, config_.vision_filter),
            config_.pitch_min, config_.pitch_max);
        const bool pitch_clipped =
            -predicted_pitch < config_.pitch_min || -predicted_pitch > config_.pitch_max;
        pitch_reachable = finite(predicted_pitch) && !pitch_clipped;
        // Keep the old vision convention: pitch position is negated, pitch velocity is not.
        const float pitch_velocity = pitch_clipped ? 0.0f : vision.pitch_vel;
        yaw_position_.mode = control::pid_mode::delta;
        pitch_position_.mode = control::pid_mode::delta;
        // Clamp once after adding velocity feedforward, as in the old two-DOF outer loop.
        yaw_position_.max_out = std::numeric_limits<float>::max();
        pitch_position_.max_out = std::numeric_limits<float>::max();
        yaw_position_.ref = yaw_reference_;
        yaw_position_.fdb = input.total_yaw;
        yaw_position_.update((vision.yaw_vel - input.gyro_yaw) * control_period_s);
        pitch_position_.ref = pitch_reference_;
        pitch_position_.fdb = input.pitch;
        pitch_position_.update((pitch_velocity - input.gyro_pitch) * control_period_s);
        yaw_speed_reference = std::clamp(vision.yaw_vel + yaw_position_.result,
            -config_.yaw_position_pid[3], config_.yaw_position_pid[3]);
        pitch_speed_reference = std::clamp(pitch_velocity + pitch_position_.result,
            -config_.pitch_position_pid[3], config_.pitch_position_pid[3]);
        yaw_effort_ff = feedforward(vision.yaw_vel, -vision.yaw_acc, config_.yaw_feedforward);
        if (!pitch_clipped)
        {
            pitch_effort_ff =
                feedforward(vision.pitch_vel, -vision.pitch_acc, config_.pitch_feedforward);
        }
    }
    else
    {
        const float dyaw =
            mouse_yaw_.update(remote.mouse_x * config_.mouse_yaw_scale, config_.manual_yaw_filter) +
            remote.right_x;
        const float dpitch = mouse_pitch_.update(remote.mouse_y * config_.mouse_pitch_scale,
                                 config_.manual_pitch_filter) +
                             remote.right_y;
        if (std::fabs(dyaw) < config_.manual_deadband)
        {
            if (!yaw_held_)
            {
                yaw_reference_ = input.total_yaw;
                yaw_held_ = true;
            }
        }
        else
        {
            yaw_held_ = false;
            yaw_reference_ = input.total_yaw - dyaw * config_.manual_angle_scale;
        }
        if (std::fabs(dpitch) < config_.manual_deadband)
        {
            if (!pitch_held_)
            {
                pitch_reference_ = input.pitch;
                pitch_held_ = true;
            }
        }
        else
        {
            pitch_held_ = false;
            pitch_reference_ = input.pitch - dpitch * config_.manual_angle_scale;
        }
        if (turn)
        {
            yaw_reference_ = input.total_yaw + pi;
            yaw_held_ = true;
        }
        if (!gimbal_only_ && (!input.lower_valid || !input.lower.inited))
        {
            yaw_reference_ = input.total_yaw;
            yaw_held_ = false;
            pitch_reference_ = 0.0f;
        }
        pitch_reference_ = std::clamp(pitch_reference_, config_.pitch_min, config_.pitch_max);
        yaw_position_.mode = measured_derivative;
        pitch_position_.mode = measured_derivative;
        yaw_position_.max_out = config_.yaw_position_pid[3];
        pitch_position_.max_out = config_.pitch_position_pid[3];
        yaw_position_.ref = yaw_reference_;
        yaw_position_.fdb = input.total_yaw;
        yaw_position_.update();
        pitch_position_.ref = pitch_reference_;
        pitch_position_.fdb = input.pitch;
        pitch_position_.update();
        yaw_speed_reference = yaw_position_.result;
        pitch_speed_reference = pitch_position_.result;
    }

    if (mode_changed)
    {
        yaw_velocity_.reset_state(yaw_speed_reference, input.gyro_yaw);
        pitch_velocity_.reset_state(pitch_speed_reference, input.gyro_pitch);
    }
    yaw_velocity_.ref = yaw_speed_reference;
    yaw_velocity_.fdb = input.gyro_yaw;
    yaw_velocity_.update();
    pitch_velocity_.ref = pitch_speed_reference;
    pitch_velocity_.fdb = input.gyro_pitch;
    pitch_velocity_.update();
    const float yaw_effort = -(yaw_velocity_.result + yaw_effort_ff);
    if (yaw_ready && finite(yaw_effort))
    {
        output.lower.yaw_cur = static_cast<int16_t>(
            std::clamp(yaw_effort, -config_.yaw_current_limit, config_.yaw_current_limit));
    }
    else
    {
        yaw_position_.reset_state(input.total_yaw, input.total_yaw);
        yaw_velocity_.reset_state(yaw_speed_reference, input.gyro_yaw);
        yaw_held_ = false;
    }
    const float gravity =
        mode_ == gimbal_mode::automatic ? config_.pitch_gravity_auto : config_.pitch_gravity_manual;
    const float pitch_effort =
        (pitch_velocity_.result + pitch_effort_ff) * config_.pitch_current_scale +
        gravity * std::cos(input.pitch);
    if (!finite(pitch_effort))
    {
        reset_gimbal(input);
        return control_output{};
    }
    output.pitch_current = static_cast<int16_t>(
        std::clamp(pitch_effort, -config_.pitch_current_limit, config_.pitch_current_limit));
    output.mode = mode_;
    output.yaw_reference = yaw_reference_;
    output.pitch_reference = pitch_reference_;

    const bool wheels_online = input.friction_left_online && input.friction_right_online &&
                               finite(input.friction_left_speed) &&
                               finite(input.friction_right_speed);
    if (shooter_ == shooter_mode::closed || !wheels_online || !input.lower_valid)
    {
        auto_fire_active_ = false;
        auto_fire_blocked_ = false;
        return output;
    }
    output.friction_enabled = true;
    output.friction_left_speed = config_.friction_speed;
    output.friction_right_speed = -config_.friction_speed;
    const float left_speed = std::fabs(input.friction_left_speed);
    const float right_speed = std::fabs(input.friction_right_speed);
    const bool wheels_ready =
        input.friction_left_speed >= config_.friction_speed * config_.friction_ready_ratio &&
        -input.friction_right_speed >= config_.friction_speed * config_.friction_ready_ratio;
    const bool heat_ready = input.lower.heatlimit > 0 &&
                            input.lower.heatnow < input.lower.heatlimit * config_.heat_fraction;
    const bool manual_fire = shooter_ == shooter_mode::warm && remote.mouse_left;
    const bool check_fire = shooter_ == shooter_mode::fire;
    const bool aim_ready =
        mode_ == gimbal_mode::automatic && yaw_ready && pitch_reachable &&
        std::fabs(yaw_reference_ - input.total_yaw) < config_.fire_angle_tolerance &&
        std::fabs(pitch_reference_ - input.pitch) < config_.fire_angle_tolerance;
    const bool automatic_request =
        output.auto_shoot && ((target_accepted && vision.mode == 2 && aim_ready) || check_fire);
    if (!automatic_request)
    {
        auto_fire_blocked_ = false;
    }
    if (!heat_ready || !wheels_ready)
    {
        auto_fire_active_ = false;
        return output;
    }
    const int8_t normal_speed =
        (!high_frequency_ || aim_rune_) ? config_.trigger_low : config_.trigger_high;
    if (manual_fire)
    {
        auto_fire_active_ = false;
        auto_fire_blocked_ = false;
        output.lower.tri_spd = remote.key.CTRL ? config_.trigger_reverse : normal_speed;
    }
    else if (output.auto_shoot)
    {
        // Loss of the target cancels the in-flight command immediately; stale mode=2 is never used.
        if (!automatic_request)
        {
            auto_fire_active_ = false;
        }
        if (!auto_fire_active_ && automatic_request && !auto_fire_blocked_)
        {
            auto_fire_active_ = true;
            auto_fire_start_ms_ = input.now_ms;
            friction_left_peak_ = left_speed;
            friction_right_peak_ = right_speed;
        }
        if (auto_fire_active_)
        {
            friction_left_peak_ = std::max(friction_left_peak_, left_speed);
            friction_right_peak_ = std::max(friction_right_peak_, right_speed);
            const bool dropped = friction_left_peak_ - left_speed >= config_.friction_drop ||
                                 friction_right_peak_ - right_speed >= config_.friction_drop;
            const bool timed_out =
                input.now_ms - auto_fire_start_ms_ >= config_.auto_fire_timeout_ms;
            if (dropped || timed_out)
            {
                auto_fire_active_ = false;
                auto_fire_blocked_ = timed_out;
                if (dropped)
                {
                    ++bullet_count_;
                }
            }
            else
            {
                output.lower.tri_spd = normal_speed;
            }
        }
    }
    else if (check_fire)
    {
        auto_fire_active_ = false;
        output.lower.tri_spd = config_.trigger_continuous;
    }
    else
    {
        auto_fire_active_ = false;
    }
    output.auto_fire = auto_fire_active_;
    output.bullet_count = bullet_count_;
    output.ui.fire = output.lower.tri_spd > 0;
    return output;
}
} // namespace big_head
