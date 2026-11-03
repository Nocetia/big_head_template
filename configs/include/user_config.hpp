#pragma once

// Generated from configs/user_config.json. Do not edit.
#include "config.hpp"
#include "ui.hpp"
#include "ps2.hpp"
#include "dmimu.hpp"
#include "bmi088.hpp"
#include "referee.hpp"
#include "buzzer.hpp"

namespace user_config
{
#if defined(STM32H723xx)
#define USER_AHRS_DEVICE 1
#define USER_REMOTER_DR16 1
#define USER_REMOTER_VT03 1
#define USER_REMOTER_PS2 0
#define USER_BIG_HEAD_APP 1
inline constexpr bool big_head_enabled = true;
inline constexpr char ahrs_device[] = "bmi088";
inline constexpr bool bmi088_heater_enabled = true;
inline constexpr imu::bmi088_thermal bmi088_thermal{45.0f, 0.059999999999999998f,
    0.050000000000000003f, 0.043999999999999997f, 0.029000000000000001f, 0.040000000000000001f,
    300.0f, 0.059999999999999998f, 100.0f, 90.0f};
inline constexpr bsp::can::config dmimu_can{bsp::can::handle::fdcan1, bsp::can::id_type::standard};
inline constexpr bsp::can::config motor_can{bsp::can::handle::fdcan1, bsp::can::id_type::standard};
inline constexpr bsp::can::config lower_can{bsp::can::handle::fdcan2, bsp::can::id_type::standard};
inline constexpr auto rfid_uart = bsp::peripherals::usart10;
inline constexpr buzzer::config rfid_buzzer{bsp::peripherals::tim12,
    bsp::pwm::channel::ch2, 333,
    100, 100};
inline constexpr auto referee_uart = bsp::usart::config{};
inline constexpr bool remoter_dr16_enabled = true;
inline constexpr bool remoter_vt03_enabled = true;
inline constexpr bool remoter_ps2_enabled = false;
inline constexpr auto ui_uart = bsp::usart::config{};
inline constexpr auto dr16_uart = bsp::peripherals::uart5;
inline constexpr auto vt03_uart = bsp::peripherals::usart1;
inline constexpr bsp::gpio::config ps2_cs{(bsp::peripherals::ps2_cs).instance,
    (bsp::peripherals::ps2_cs).pin, bsp::active_level::low};
inline constexpr auto ps2_spi = bsp::spi::handle::spi1;
#elif defined(STM32F407xx)
#define USER_AHRS_DEVICE 1
#define USER_REMOTER_DR16 1
#define USER_REMOTER_VT03 1
#define USER_REMOTER_PS2 0
#define USER_BIG_HEAD_APP 0
inline constexpr bool big_head_enabled = false;
inline constexpr char ahrs_device[] = "bmi088";
inline constexpr bool bmi088_heater_enabled = false;
inline constexpr imu::bmi088_thermal bmi088_thermal{45.0f, 0.059999999999999998f,
    0.050000000000000003f, 0.043999999999999997f, 0.029000000000000001f, 0.040000000000000001f,
    650.0f, 0.080000000000000002f, 10.0f, 900.0f};
inline constexpr bsp::can::config dmimu_can{bsp::can::handle::can1, bsp::can::id_type::standard};
inline constexpr bsp::can::config motor_can{bsp::can::handle::none, bsp::can::id_type::standard};
inline constexpr bsp::can::config lower_can{bsp::can::handle::none, bsp::can::id_type::standard};
inline constexpr auto rfid_uart = bsp::peripherals::usart1;
inline constexpr buzzer::config rfid_buzzer{bsp::peripherals::tim4,
    bsp::pwm::channel::ch3, 333,
    100, 100};
inline constexpr auto referee_uart = bsp::usart::config{};
inline constexpr bool remoter_dr16_enabled = true;
inline constexpr bool remoter_vt03_enabled = true;
inline constexpr bool remoter_ps2_enabled = false;
inline constexpr auto ui_uart = bsp::usart::config{};
inline constexpr auto dr16_uart = bsp::peripherals::usart3;
inline constexpr auto vt03_uart = bsp::peripherals::usart6;
inline constexpr bsp::gpio::config ps2_cs{(bsp::gpio::config{}).instance,
    (bsp::gpio::config{}).pin, bsp::active_level::low};
inline constexpr auto ps2_spi = bsp::spi::handle::none;
#else
#error Unsupported board
#endif

namespace usb
{
inline constexpr bool enabled = true;
} // namespace usb

inline constexpr imu::bmi088_heater_config bmi088_heater{
    125, 500,
    2000, 0.5f, 2};
inline constexpr ULONG bmi088_heater_stack_size = 2048;

inline constexpr imu::bmi088_config bmi088{board::bmi088::spi, board::bmi088::acc_cs,
    board::bmi088::gyro_cs};
namespace application
{
inline constexpr math::vector3 imu_offset{0.0f, 0.0f, 0.0f};
namespace big_head
{
inline constexpr uint32_t control_period_ms = 1;
inline constexpr uint32_t control_priority = 4;
inline constexpr ULONG control_stack_size = 8192;
inline constexpr uint32_t usb_period_ms = 2;
inline constexpr uint32_t usb_write_timeout_ms = 5;
inline constexpr uint32_t usb_priority = 11;
inline constexpr ULONG usb_stack_size = 2048;
inline constexpr uint32_t vt03_baud_rate = 921600;
inline constexpr uint32_t imu_timeout_ms = 20;
inline constexpr uint32_t vision_timeout_ms = 100;
inline constexpr uint32_t lower_timeout_ms = 100;
inline constexpr uint32_t motor_timeout_ms = 100;
inline constexpr uint32_t recovery_period_ms = 100;
inline constexpr uint32_t auto_fire_timeout_ms = 1000;
inline constexpr uint16_t pitch_id = 517;
inline constexpr uint16_t friction_left_id = 513;
inline constexpr uint16_t friction_right_id = 514;
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
inline constexpr float pitch_min = -0.59999999999999998f;
inline constexpr float pitch_max = 0.40000000000000002f;
inline constexpr float manual_deadband = 0.0050000000000000001f;
inline constexpr float manual_angle_scale = 0.050000000000000003f;
inline constexpr float mouse_yaw_scale = 0.025000000000000001f;
inline constexpr float mouse_pitch_scale = 0.01f;
inline constexpr float keyboard_slew_per_tick = 0.002f;
inline constexpr float friction_gear_ratio = 1.0f;
inline constexpr float friction_speed = 600.0f;
inline constexpr float friction_ready_ratio = 0.90000000000000002f;
inline constexpr float friction_drop = 6.0f;
inline constexpr float heat_fraction = 0.69999999999999996f;
inline constexpr float fire_angle_tolerance = 0.01f;
inline constexpr int8_t trigger_low = 4;
inline constexpr int8_t trigger_high = 8;
inline constexpr int8_t trigger_continuous = 16;
inline constexpr int8_t trigger_reverse = -8;
inline constexpr bool auto_aim_default = true;
inline constexpr bool auto_shoot_default = true;
inline constexpr bool gimbal_only_default = true;
// prediction seconds, inertia, damping, command/torque scale.
inline constexpr float yaw_feedforward[4]{0.0089999999999999993f, 0.02f, 0.61511340000000003f, 1.3500000000000001f};
inline constexpr float pitch_feedforward[4]{0.0f, 0.02f, 0.61511340000000003f, 1.3500000000000001f};
// Scalar Kalman filters: process noise Q, measurement noise R.
inline constexpr float manual_yaw_filter[2]{0.10000000000000001f, 0.54300000000000004f};
inline constexpr float manual_pitch_filter[2]{0.001f, 0.54300000000000004f};
inline constexpr float vision_filter[2]{0.001f, 0.54300000000000004f};
} // namespace big_head
} // namespace application

namespace ahrs
{
inline constexpr uint32_t thread_priority = 3;
inline constexpr ULONG bmi088_stack_size = 8192;
inline constexpr ULONG dmimu_stack_size = 2048;
inline constexpr ULONG sample_period_ticks = 1;
inline constexpr float max_dt = 0.02f;
inline constexpr uint32_t retry_period_ms = 1000;
inline constexpr uint32_t warmup_timeout_ms = 60000;
inline constexpr bool calibrate_on_start = false;
inline constexpr ULONG dmimu_wait_ticks = 1;
inline constexpr uint32_t dmimu_request_period_ms = 10;
inline constexpr uint32_t dmimu_queue_frames = 32;
namespace eskf
{
inline constexpr float gyro_noise = 0.00029999999999999997f;
inline constexpr float gyro_bias_walk = 2.0000000000000002e-05f;
inline constexpr float accel_noise = 0.00059999999999999995f;
inline constexpr float accel_magnitude_gain = 1.0f;
inline constexpr float nis_gate = 9.2100000000000009f;
inline constexpr float reject_timeout = 0.20000000000000001f;
inline constexpr float impact_jerk = 400.0f;
inline constexpr float impact_hold = 0.050000000000000003f;
inline constexpr float static_accel_tolerance = 0.050000000000000003f;
inline constexpr float static_gyro_tolerance = 0.02f;
inline constexpr float static_filter_time_constant = 0.050000000000000003f;
inline constexpr float zero_rate_min_time = 1.0f;
inline constexpr float initial_tilt_stddev = 0.10000000000000001f;
inline constexpr float initial_yaw_stddev = 0.001f;
inline constexpr float initial_bias_stddev = 0.01f;
inline constexpr float min_bias_stddev = 0.00020000000000000001f;
} // namespace eskf
} // namespace ahrs

inline constexpr math::vector3 bmi088_gyro_offset{-0.0052809934869999997f, -0.000237223741f, -0.000647540528f};
inline constexpr math::vector3 bmi088_accel_offset{0.0038458286072392397f, 0.0064703959499354804f, 0.014968990490337293f};

inline constexpr ::referee::config referee{
    referee_uart, 6, 1, 1000};
inline constexpr ULONG referee_stack_size = 2048;
namespace remoter
{
inline constexpr uint32_t thread_priority = 4;
inline constexpr ULONG stack_size = 2048;
inline constexpr ULONG period_ticks = 1;
inline constexpr uint32_t offline_timeout_ms = 100;
inline constexpr uint32_t ps2_period_ms = 10;
}

inline constexpr ui::config ui{ui_uart.instance, 100};
inline constexpr imu::dmimu::config dmimu{dmimu_can.instance, 1, 17,
    imu::dmimu::mode::active, 20, 100};
// HAL's DR16 word length includes the parity bit.
inline constexpr bsp::usart::line_config dr16_line{100000, bsp::usart::word_length::bits_9,
    bsp::usart::stop_bits::one, bsp::usart::parity::even, false, true};
inline constexpr bsp::usart::line_config vt03_line{115200, bsp::usart::word_length::bits_8,
    bsp::usart::stop_bits::one, bsp::usart::parity::none, false, true};
inline constexpr float ps2_deadzone = 0.02f;
inline constexpr uint32_t ps2_spi_timeout_ms = 20;
} // namespace user_config

namespace user_config::rfid
{
inline constexpr uint8_t address = 32;
inline constexpr uint8_t block = 4;
inline constexpr uint32_t baud = 9600;
inline constexpr uint32_t priority = 8;
inline constexpr uint32_t stack_size = 2048;
inline constexpr uint32_t startup_ms = 1000;
inline constexpr uint32_t poll_ms = 120;
inline constexpr uint32_t timeout_ms = 300;
inline constexpr uint32_t recovery_ms = 1000;
inline constexpr auto buzzer = user_config::rfid_buzzer;
} // namespace user_config::rfid
