#include "bsp_can.hpp"
#include "bsp_dwt.hpp"
#include "m3508.hpp"
#include "dji_manager.hpp"
#include "main.h"
#include "smc.hpp"
#include "tx_api.h"
#include "user_config.hpp"

#include <cmath>
#include <cstdint>
volatile float target_velocity_debug = 1.0f;
volatile float debug_velocity_fdb = 0.0f;

namespace smc_motor_test
{
namespace cfg
{
inline constexpr uint16_t feedback_id = 0x201;
inline constexpr uint16_t command_id = 0x200;
inline constexpr uint32_t control_period_ms = 1;
inline constexpr float control_period_s = control_period_ms * 0.001f;
inline constexpr uint32_t feedback_timeout_ms = 100;
inline constexpr uint32_t recovery_period_ms = 100;
inline constexpr uint32_t arming_delay_ms = 500;
inline constexpr float target_velocity = 1.0f; // M3508 output shaft, rad/s.
inline constexpr float current_limit = 1500.0f; // C620 protocol units.
inline constexpr UINT thread_priority = 4;
inline constexpr ULONG thread_stack_size = 2048;
} // namespace cfg

static_assert(TX_TIMER_TICKS_PER_SECOND == 1000, "SMC test requires a 1 kHz ThreadX tick");

// This first bench test uses inertia as an effective raw-current scale. Replace it with an
// identified inertia and a calibrated torque-to-current conversion for physical-unit control.
control::smc_config controller_config()
{
    control::smc_config result;
    result.mode = control::smc_mode::velocity;
    result.inertia = 20.0f;
    result.reaching_gain = 4.0f;
    result.surface_gain = 2.0f;
    result.switching_gain = 1.0f;
    result.boundary_layer = 2.0f;
    result.saturation_limit = 1.0f;
    result.output_limit = cfg::current_limit;
    result.velocity_integral_limit = 2.0f;
    return result;
}

struct diagnostics
{
    float target_velocity = cfg::target_velocity;
    float velocity = 0.0f;
    float surface = 0.0f;
    float smc_output = 0.0f;
    int16_t current_command = 0;
    uint32_t feedback_updates = 0;
    uint32_t restart_attempts = 0;
    uint32_t overruns = 0;
    bool feedback_online = false;
    bool armed = false;
    float cur_fdb = 0.0f;
};

diagnostics diagnostic;

namespace
{
motor::dji::m3508 wheel;
motor::dji::manager motors;
control::smc controller;

TX_THREAD control_thread;
alignas(8) uint8_t control_stack[cfg::thread_stack_size];

void control_entry(ULONG)
{
    using pnxtypes::status;
    const auto bus = user_config::motor_can.instance;
    uint32_t last_feedback_ms = 0;
    uint32_t online_since_ms = 0;
    uint32_t last_restart_ms = HAL_GetTick() - cfg::recovery_period_ms;
    bool feedback_valid = false;
    bool bus_ready = true;
    ULONG deadline = tx_time_get();

    for (;;)
    {
        // target_velocity=0;
        const auto now = HAL_GetTick();
        bool bus_error = bsp::can::current_status(bus) != status::ok;
        auto* errors = bsp::can::err_sem(bus);
        for (unsigned i = 0; errors != nullptr && i < 8; ++i)
        {
            if (tx_semaphore_get(errors, TX_NO_WAIT) != TX_SUCCESS)
            {
                break;
            }
            bus_error = true;
        }
        if (bus_error)
        {
            bus_ready = false;
            feedback_valid = false;
            controller.reset();
        }
        if (!bus_ready && uint32_t(now - last_restart_ms) >= cfg::recovery_period_ms)
        {
            last_restart_ms = now;
            ++diagnostic.restart_attempts;
            bus_ready = bsp::can::restart(bus) == status::ok;
            if (bus_ready)
            {
                // Do not let a frame buffered before the restart participate in re-arming.
                (void)motors.update_feedback(wheel);
            }
        }

        if (motors.update_feedback(wheel) && bus_ready)
        {
            if (!feedback_valid)
            {
                online_since_ms = now;
            }
            feedback_valid = true;
            last_feedback_ms = now;
            ++diagnostic.feedback_updates;
        }
        if (feedback_valid && uint32_t(now - last_feedback_ms) >= cfg::feedback_timeout_ms)
        {
            feedback_valid = false;
            controller.reset();
        }

        const bool armed = bus_ready && feedback_valid &&
                           uint32_t(now - online_since_ms) >= cfg::arming_delay_ms;
        float smc_output = 0.0f;
        int16_t current_command = 0;
        if (armed)
        {
            smc_output = controller.update_velocity(
                target_velocity_debug , wheel.fdb.velocity, cfg::control_period_s);
            if (controller.state().valid)
            {
                current_command = static_cast<int16_t>(std::lround(smc_output));
                wheel.motor_mode = motor::mode::current;
                wheel.cmd.current = current_command;
            }
            else
            {
                wheel.motor_mode = motor::mode::relax;
                wheel.cmd.current = 0;
            }
        }
        else
        {
            controller.reset();
            wheel.motor_mode = motor::mode::relax;
            wheel.cmd.current = 0;
        }

        motors.update();
        if (bus_ready)
        {
            motors.transmit(static_cast<uint32_t>(bsp::dwt::timeline_us()));
        }

        diagnostic.velocity = wheel.fdb.velocity;
        debug_velocity_fdb=wheel.fdb.velocity;
        diagnostic.cur_fdb = wheel.fdb.torque_current_raw;
        diagnostic.surface = controller.state().surface;
        diagnostic.smc_output = smc_output;
        diagnostic.current_command = current_command;
        diagnostic.feedback_online = feedback_valid;
        diagnostic.armed = armed;

        deadline += cfg::control_period_ms;
        const auto remaining = static_cast<int32_t>(deadline - tx_time_get());
        if (remaining > 0)
        {
            tx_thread_sleep(static_cast<ULONG>(remaining));
        }
        else
        {
            ++diagnostic.overruns;
            tx_thread_sleep(1);
            deadline = tx_time_get();
        }
        
    }
}
} // namespace
} // namespace smc_motor_test

extern "C" void app_start()
{
    using namespace smc_motor_test;
    const auto bus = user_config::motor_can.instance;
    if (bus == bsp::can::handle::none || bsp::dwt::init() != pnxtypes::status::ok ||
        !controller.configure(controller_config()) ||
        !motors.register_motor(wheel, bus, cfg::feedback_id) ||
        !motors.set_tx_period(bus, cfg::command_id, cfg::control_period_ms * 1000) ||
        bsp::can::init(user_config::motor_can) != pnxtypes::status::ok)
    {
        Error_Handler();
        return;
    }
    motors.set_period(cfg::control_period_s);
    wheel.motor_mode = motor::mode::relax;
    wheel.cmd.current = 0;
    if (tx_thread_create(&control_thread, const_cast<CHAR*>("m3508_smc"), control_entry, 0,
            control_stack, sizeof(control_stack), cfg::thread_priority, cfg::thread_priority,
            TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        Error_Handler();
    }
}
