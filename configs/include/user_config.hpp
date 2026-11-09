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
inline constexpr char ahrs_device[] = "bmi088";
inline constexpr bool bmi088_heater_enabled = true;
inline constexpr imu::bmi088_thermal bmi088_thermal{45.0f, 0.059999999999999998f,
    0.050000000000000003f, 0.043999999999999997f, 0.029000000000000001f, 0.040000000000000001f,
    300.0f, 0.059999999999999998f, 100.0f, 90.0f};
inline constexpr bsp::can::config dmimu_can{bsp::can::handle::fdcan1, bsp::can::id_type::standard};
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
inline constexpr char ahrs_device[] = "bmi088";
inline constexpr bool bmi088_heater_enabled = false;
inline constexpr imu::bmi088_thermal bmi088_thermal{45.0f, 0.059999999999999998f,
    0.050000000000000003f, 0.043999999999999997f, 0.029000000000000001f, 0.040000000000000001f,
    650.0f, 0.080000000000000002f, 10.0f, 900.0f};
inline constexpr bsp::can::config dmimu_can{bsp::can::handle::can1, bsp::can::id_type::standard};
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
inline constexpr float zero_rate_exit_gyro_tolerance = 0.050000000000000003f;
inline constexpr float zero_rate_nis_gate = 11.34f;
inline constexpr float zero_rate_attitude_ramp_time = 0.5f;
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
