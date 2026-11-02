#pragma once

#include "pid.hpp"
#include "protocol.hpp"
#include "remoter.hpp"
#include <cstdint>

namespace big_head
{
enum class remote_source : uint8_t
{
    none,
    dr16,
    vt03,
};

enum class gimbal_mode : uint8_t
{
    relax,
    manual,
    automatic,
};

// Values come from the generated application configuration. PID gains assume 1 kHz.
struct control_parameters
{
    float yaw_position_pid[5];
    float yaw_velocity_pid[5];
    float pitch_position_pid[5];
    float pitch_velocity_pid[5];
    float yaw_feedforward[4]; // prediction seconds, inertia, damping, command / torque
    float pitch_feedforward[4];
    float manual_yaw_filter[2]; // process and measurement covariance
    float manual_pitch_filter[2];
    float vision_filter[2];
    float yaw_current_limit;
    float pitch_current_limit;
    float pitch_current_scale;
    float pitch_gravity_manual;
    float pitch_gravity_auto;
    float pitch_min;
    float pitch_max;
    float manual_deadband;
    float manual_angle_scale;
    float mouse_yaw_scale;
    float mouse_pitch_scale;
    float keyboard_slew_per_tick;
    float friction_speed;
    float friction_ready_ratio;
    float friction_drop;
    float heat_fraction;
    float fire_angle_tolerance;
    int8_t trigger_low;
    int8_t trigger_high;
    int8_t trigger_continuous;
    int8_t trigger_reverse;
    uint32_t auto_fire_timeout_ms;
    bool auto_aim_default;
    bool auto_shoot_default;
    bool gimbal_only_default;
};

// The runtime validates freshness and adapts sensor axes before calling update().
// Angles are radians, angular velocities are rad/s, friction speeds are rotor rad/s.
struct control_input
{
    uint32_t now_ms = 0;
    remotes::reading remote{};
    remote_source source = remote_source::none;
    bool remote_new_frame = false;
    bool remote_valid = false;
    float pitch = 0.0f;
    float yaw = 0.0f;
    float total_yaw = 0.0f;
    float gyro_pitch = 0.0f;
    float gyro_yaw = 0.0f;
    bool imu_valid = false;
    vision_command vision{};
    bool vision_valid = false;
    lower_status lower{};
    bool lower_valid = false;
    float friction_left_speed = 0.0f;
    float friction_right_speed = 0.0f;
    bool pitch_online = false;
    bool friction_left_online = false;
    bool friction_right_online = false;
};

struct control_output
{
    int16_t pitch_current = 0;
    float friction_left_speed = 0.0f;
    float friction_right_speed = 0.0f;
    bool friction_enabled = false;
    lower_command lower{};
    lower_ui ui{};
    gimbal_mode mode = gimbal_mode::relax;
    bool armed = false;
    bool auto_fire = false;
    bool auto_aim = false;
    bool auto_shoot = false;
    bool aim_rune = false;
    float pitch_reference = 0.0f;
    float yaw_reference = 0.0f;
    uint16_t bullet_count = 0;
};

// Owned exclusively by the 1 kHz control thread; no I/O, allocation or background work.
class controller
{
public:
    explicit controller(const control_parameters& parameters);
    control_output update(const control_input& input);

private:
    enum class shooter_mode : uint8_t
    {
        closed,
        warm,
        fire,
    };

    struct scalar_filter
    {
        float value = 0.0f;
        float covariance = 0.02f;
        float update(float sample, const float parameters[2]);
        void reset(float sample = 0.0f);
    };

    void reset_gimbal(const control_input& input);
    void reset_remote(const control_input& input);

    const control_parameters config_;
    control::pid yaw_position_;
    control::pid yaw_velocity_;
    control::pid pitch_position_;
    control::pid pitch_velocity_;
    scalar_filter mouse_yaw_;
    scalar_filter mouse_pitch_;
    scalar_filter vision_yaw_;
    scalar_filter vision_pitch_;
    remote_source source_ = remote_source::none;
    remotes::reading previous_remote_{};
    gimbal_mode mode_ = gimbal_mode::relax;
    shooter_mode shooter_ = shooter_mode::closed;
    bool armed_ = false;
    bool relaxed_seen_ = false;
    bool high_frequency_ = true;
    bool aim_rune_ = false;
    bool gimbal_only_ = false;
    bool yaw_held_ = false;
    bool pitch_held_ = false;
    float yaw_reference_ = 0.0f;
    float pitch_reference_ = 0.0f;
    float keyboard_vx_ = 0.0f;
    float keyboard_vy_ = 0.0f;
    int8_t leg_level_ = 0;
    bool auto_fire_active_ = false;
    bool auto_fire_blocked_ = false;
    float friction_left_peak_ = 0.0f;
    float friction_right_peak_ = 0.0f;
    uint32_t auto_fire_start_ms_ = 0;
    uint16_t bullet_count_ = 0;
};
} // namespace big_head
