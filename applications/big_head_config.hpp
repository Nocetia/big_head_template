#pragma once

#include "bsp_can.hpp"

#include <cstdint>

namespace big_head::config
{
inline constexpr bool big_head_enabled = true;
inline constexpr bsp::can::config motor_can{
    bsp::can::handle::fdcan1, bsp::can::id_type::standard};
inline constexpr bsp::can::config lower_can{
    bsp::can::handle::fdcan2, bsp::can::id_type::standard};

inline constexpr std::uint32_t control_period_ms = 1;
inline constexpr std::uint32_t control_priority = 4;
inline constexpr std::uint32_t control_stack_size = 8192;
inline constexpr std::uint32_t usb_period_ms = 2;
inline constexpr std::uint32_t usb_write_timeout_ms = 5;
inline constexpr std::uint32_t usb_priority = 11;
inline constexpr std::uint32_t usb_stack_size = 2048;
inline constexpr std::uint32_t vt03_baud_rate = 921600;
inline constexpr std::uint32_t imu_timeout_ms = 20;
inline constexpr std::uint32_t vision_timeout_ms = 100;
inline constexpr std::uint32_t lower_timeout_ms = 100;
inline constexpr std::uint32_t motor_timeout_ms = 100;
inline constexpr std::uint32_t recovery_period_ms = 100;
inline constexpr std::uint32_t auto_fire_timeout_ms = 1000;

inline constexpr std::uint16_t pitch_id = 0x205;
inline constexpr std::uint16_t friction_left_id = 0x201;
inline constexpr std::uint16_t friction_right_id = 0x202;

// PID tuples: kp, ki, kd, max_out, max_iout; tuned at control_period_ms = 1.
inline constexpr float yaw_position_pid[5]{140.0f, 0.0f, 3000.0f, 500.0f, 10.0f};
inline constexpr float yaw_velocity_pid[5]{4000.0f, 0.0f, 10000.0f, 25000.0f, 100.0f};
inline constexpr float pitch_position_pid[5]{80.0f, 0.0f, 800.0f, 500.0f, 50.0f};
inline constexpr float pitch_velocity_pid[5]{200.0f, 0.0f, 0.0f, 500.0f, 50.0f};
inline constexpr float friction_velocity_pid[5]{100.0f, 0.0f, 0.0f, 15000.0f, 3.0f};

inline constexpr float yaw_current_limit = 25000.0f;
inline constexpr float pitch_current_limit = 15000.0f;
inline constexpr float pitch_current_scale = 50.0f;
inline constexpr float pitch_gravity_manual = 1750.0f;
inline constexpr float pitch_gravity_auto = 2000.0f;
inline constexpr float pitch_min = -0.6f;
inline constexpr float pitch_max = 0.4f;
inline constexpr float manual_deadband = 0.005f;
inline constexpr float manual_angle_scale = 0.05f;
inline constexpr float mouse_yaw_scale = 0.025f;
inline constexpr float mouse_pitch_scale = 0.01f;
inline constexpr float keyboard_slew_per_tick = 0.002f;
inline constexpr float friction_speed = 600.0f;
inline constexpr float friction_ready_ratio = 0.9f;
inline constexpr float friction_drop = 6.0f;
inline constexpr float heat_fraction = 0.7f;
inline constexpr float fire_angle_tolerance = 0.01f;

inline constexpr std::int8_t trigger_low = 4;
inline constexpr std::int8_t trigger_high = 8;
inline constexpr std::int8_t trigger_continuous = 16;
inline constexpr std::int8_t trigger_reverse = -8;
inline constexpr bool auto_aim_default = true;
inline constexpr bool auto_shoot_default = true;
inline constexpr bool gimbal_only_default = true;

// Feedforward tuples: prediction seconds, inertia, damping, command/torque scale.
inline constexpr float yaw_feedforward[4]{0.009f, 0.02f, 0.6151134f, 1.35f};
inline constexpr float pitch_feedforward[4]{0.0f, 0.02f, 0.6151134f, 1.35f};

// Scalar Kalman filters: process noise Q, measurement noise R.
inline constexpr float manual_yaw_filter[2]{0.1f, 0.543f};
inline constexpr float manual_pitch_filter[2]{0.001f, 0.543f};
inline constexpr float vision_filter[2]{0.001f, 0.543f};

static_assert(control_period_ms == 1, "PID tuning requires a 1 ms control period");
static_assert(motor_can.instance != bsp::can::handle::none &&
                  lower_can.instance != bsp::can::handle::none &&
                  motor_can.instance != lower_can.instance,
    "motor and lower-controller CAN resources must be distinct");
static_assert(pitch_id != friction_left_id && pitch_id != friction_right_id &&
                  friction_left_id != friction_right_id,
    "motor feedback IDs must be unique");
static_assert(pitch_min < pitch_max, "pitch_min must be less than pitch_max");
static_assert(friction_velocity_pid[3] > 0.0f && friction_velocity_pid[3] <= 16384.0f,
    "M3508 command must fit the DJI current range");
static_assert(yaw_current_limit > 0.0f && yaw_current_limit <= 32767.0f,
    "yaw current command must fit the DJI current range");
static_assert(pitch_current_limit > 0.0f && pitch_current_limit <= 15000.0f,
    "GM6020 current command must not exceed its configured limit");
static_assert(manual_yaw_filter[1] > 0.0f && manual_pitch_filter[1] > 0.0f &&
                  vision_filter[1] > 0.0f,
    "Kalman measurement noise must be positive");
} // namespace big_head::config
