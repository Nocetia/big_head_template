#include "big_head_config.hpp"
#include "control.hpp"
#include "remote_stream.hpp"
#include "user_config.hpp"
#include "attitude.hpp"
#include "bsp_dwt.hpp"
#include "bsp_usb.hpp"
#include "dji_manager.hpp"
#include "gm6020.hpp"
#include "m3508.hpp"
#include "smc.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

#define smc_test_on_friction

namespace big_head
{
namespace cfg = config;
using pnxtypes::status;

// Visible in the debugger; counters are written only by their owning thread/IRQ.
struct diagnostics
{
    uint32_t control_overruns = 0;
    uint32_t lower_tx_failures = 0;
    uint32_t motor_restarts = 0;
    uint32_t lower_restarts = 0;
    uint32_t usb_tx_failures = 0;
    uint32_t usb_partial_writes = 0;
    uint32_t pitch_saturations = 0;
};

diagnostics diagnostic;

// Single-writer control snapshot for the debugger; angles are rad, current is protocol units.
struct yaw_debug_state
{
    float reference = 0.0f;
    float feedback = 0.0f;
    int16_t current = 0;
    uint32_t sequence = 0;
};

yaw_debug_state yaw_debug;

namespace
{
static_assert(TX_TIMER_TICKS_PER_SECOND == 1000, "The migrated gains require a 1 kHz tick");

template <typename T> struct sample
{
    T data{};
    uint32_t received_ms = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

template <typename T> bool fresh(const sample<T>& value, uint32_t now, uint32_t timeout)
{
    return value.valid && uint32_t(now - value.received_ms) < timeout;
}

struct can_bytes
{
    uint8_t bytes[8]{};
};

// Lower-controller parsing runs outside the IRQ in the control thread. DJI feedback is buffered
// by motor::dji::manager and consumed by the same thread.
sample<can_bytes> lower_received;
sample<bool> motor_received[3];

struct uart_bytes
{
    uint8_t data[256]{};
    uint16_t size = 0;
    uint32_t received_ms = 0;
    bool reset = false;
};

struct remote_receiver
{
    bsp::usart::config resource;
    bsp::usart::line_config line;
    memory::buffer<64>& dma;
    remote_stream parser;
    uart_bytes queued{};
    sample<remotes::reading> latest{};
    uint32_t last_attempt_ms = 0;
    uint32_t overflows = 0;
    uint32_t restarts = 0;
    bool enabled;
    bool started = false;
};

memory::buffer<64> dr16_dma RAM_D1_BSS;
memory::buffer<64> vt03_dma RAM_D1_BSS;

bsp::usart::line_config vt03_line()
{
    auto line = user_config::vt03_line;
    line.baud_rate = cfg::vt03_baud_rate;//这里转了一下从默认的115200到921600
    return line;
}

remote_receiver remotes_rx[]{{user_config::dr16_uart, user_config::dr16_line, dr16_dma,
                                 remote_stream{false}, {}, {}, 0, 0, 0,
                                 user_config::remoter_dr16_enabled},
    {user_config::vt03_uart, vt03_line(), vt03_dma, remote_stream{true}, {}, {}, 0, 0, 0,
        user_config::remoter_vt03_enabled}};

struct can_bus
{
    bsp::can::config resource;
    uint32_t last_attempt_ms = 0;
    bool ready = false;
};

can_bus motor_bus{cfg::motor_can};
can_bus lower_bus{cfg::lower_can};

motor::dji::m3508 friction_left{motor::dji::m3508::gear_ratio::direct_drive};
motor::dji::m3508 friction_right{motor::dji::m3508::gear_ratio::direct_drive};
motor::dji::gm6020 pitch;

motor::dji::manager motors;
msg::subscriber<imu::attitude::state> attitude_reader;
sample<vision_command> vision_received;
sample<vision_telemetry> telemetry;
vision_parser vision_decoder;

#ifdef smc_test_on_friction
control::smc_config friction_smc_config()
{
    control::smc_config result;
    result.mode = control::smc_mode::velocity;
    result.inertia = 0.1f;
    result.reaching_gain = 0.5f;
    result.surface_gain = 0.5f;
    result.switching_gain = 0.5f;
    result.boundary_layer = 0.5f;
    result.saturation_limit = 1.0f;
    result.output_limit = 1500.0f;
    result.velocity_integral_limit = 2.0f;
    return result;
}

control::smc friction_left_smc;
control::smc friction_right_smc;
#else
void configure_friction_motor(motor::dji::m3508& instance)
{
    instance.speed_pid = control::pid(cfg::friction_velocity_pid[0],
        cfg::friction_velocity_pid[1], cfg::friction_velocity_pid[2],
        cfg::friction_velocity_pid[3], cfg::friction_velocity_pid[4]);
}
#endif

TX_THREAD control_thread;
TX_THREAD usb_thread;
alignas(8) uint8_t control_stack[cfg::control_stack_size];
alignas(8) uint8_t usb_stack[cfg::usb_stack_size];

control_parameters control_settings()
{
    control_parameters result{};
    std::copy_n(cfg::yaw_position_pid, 5, result.yaw_position_pid);
    std::copy_n(cfg::yaw_velocity_pid, 5, result.yaw_velocity_pid);
    std::copy_n(cfg::pitch_position_pid, 5, result.pitch_position_pid);
    std::copy_n(cfg::pitch_velocity_pid, 5, result.pitch_velocity_pid);
    std::copy_n(cfg::yaw_feedforward, 4, result.yaw_feedforward);
    std::copy_n(cfg::pitch_feedforward, 4, result.pitch_feedforward);
    std::copy_n(cfg::manual_yaw_filter, 2, result.manual_yaw_filter);
    std::copy_n(cfg::manual_pitch_filter, 2, result.manual_pitch_filter);
    std::copy_n(cfg::vision_filter, 2, result.vision_filter);//气死爸爸了，manual pitch的参数不对
    result.yaw_current_limit = cfg::yaw_current_limit;
    result.pitch_current_limit = cfg::pitch_current_limit;
    result.pitch_current_scale = cfg::pitch_current_scale;
    result.pitch_gravity_manual = cfg::pitch_gravity_manual;
    result.pitch_gravity_auto = cfg::pitch_gravity_auto;
    result.pitch_min = cfg::pitch_min;
    result.pitch_max = cfg::pitch_max;
    result.manual_deadband = cfg::manual_deadband;
    result.manual_angle_scale = cfg::manual_angle_scale;
    result.mouse_yaw_scale = cfg::mouse_yaw_scale;
    result.mouse_pitch_scale = cfg::mouse_pitch_scale;
    result.keyboard_slew_per_tick = cfg::keyboard_slew_per_tick;
    result.friction_speed = cfg::friction_speed;
    result.friction_ready_ratio = cfg::friction_ready_ratio;
    result.friction_drop = cfg::friction_drop;
    result.heat_fraction = cfg::heat_fraction;
    result.fire_angle_tolerance = cfg::fire_angle_tolerance;//开火区间
    result.trigger_low = cfg::trigger_low;
    result.trigger_high = cfg::trigger_high;
    result.trigger_continuous = cfg::trigger_continuous;//trgger_spd
    result.trigger_reverse = cfg::trigger_reverse;
    result.auto_fire_timeout_ms = cfg::auto_fire_timeout_ms;
    result.auto_aim_default = cfg::auto_aim_default;
    result.auto_shoot_default = cfg::auto_shoot_default;
    result.gimbal_only_default = cfg::gimbal_only_default;
    return result;
}

void can_receive(bsp::can::handle bus, const bsp::can::rx_frame& frame)
{
    if (bus != lower_bus.resource.instance || frame.id != lower_status_id || frame.len != 8 ||
        frame.remote || frame.id_kind != bsp::can::id_type::standard ||
        frame.format != bsp::can::frame_mode::classic)
    {
        return;
    }
    const auto now = HAL_GetTick();
    const auto mask = __get_PRIMASK();
    __disable_irq();
    std::memcpy(lower_received.data.bytes, frame.data, sizeof(lower_received.data.bytes));
    lower_received.received_ms = now;
    ++lower_received.sequence;
    lower_received.valid = true;
    __set_PRIMASK(mask);
}

void uart_receive(bsp::usart::handle instance, uint8_t* bytes, uint32_t size)
{
    for (auto& remote : remotes_rx)
    {
        if (!remote.enabled || remote.resource.instance != instance)
        {
            continue;
        }
        const auto now = HAL_GetTick();
        const auto mask = __get_PRIMASK();
        __disable_irq();
        auto& queued = remote.queued;
        if (uint32_t(now - queued.received_ms) >= user_config::remoter::offline_timeout_ms)
        {
            queued.size = 0;
            queued.reset = true;
        }
        queued.received_ms = now;
        if (size > sizeof(queued.data) - queued.size)
        {
            queued.size = 0;
            queued.reset = true;
            ++remote.overflows;
        }
        else
        {
            std::memcpy(queued.data + queued.size, bytes, size);
            queued.size += static_cast<uint16_t>(size);
        }
        __set_PRIMASK(mask);
        return;
    }
}

void usb_receive(const uint8_t* bytes, uint32_t size)
{
    // The BSP USB RX thread is the sole parser owner and uses its 2048-byte stack.
    static uint32_t last_chunk_ms = 0;
    const auto now = HAL_GetTick();
    if (uint32_t(now - last_chunk_ms) >= cfg::vision_timeout_ms)
    {
        vision_decoder.reset();
    }
    last_chunk_ms = now;
    for (uint32_t i = 0; i < size; ++i)
    {
        vision_command decoded;
        if (vision_decoder.push(bytes[i], decoded))
        {
            const auto mask = __get_PRIMASK();
            __disable_irq();
            vision_received.data = decoded;
            vision_received.received_ms = now;
            ++vision_received.sequence;
            vision_received.valid = true;
            __set_PRIMASK(mask);
        }
    }
}

bool start_remote(remote_receiver& remote)
{
    // BSP mode selects TX behavior; DMA RX works in block mode, including UART5
    // which has no TX DMA. configure() invalidates init(), so initialize again.
    if (bsp::usart::init(remote.resource) != status::ok ||
        bsp::usart::configure(remote.resource.instance, remote.line) != status::ok ||
        bsp::usart::init(remote.resource) != status::ok)
    {
        return false;
    }
    std::memset(remote.dma.data(), 0, remote.dma.capacity());
    return bsp::usart::start_rx_to_idle(remote.resource.instance, remote.dma,
               bsp::usart::rx_callback::bind<uart_receive>()) == status::ok;
}

void read_remote(remote_receiver& remote)
{
    if (!remote.enabled)
    {
        return;
    }
    uart_bytes received;
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        received = remote.queued;
        remote.queued.size = 0;
        remote.queued.reset = false;
        __set_PRIMASK(mask);
    }
    const auto now = HAL_GetTick();
    if (received.reset)
    {
        remote.parser.reset();
        remote.latest.valid = false;
    }
    if (uint32_t(now - received.received_ms) < user_config::remoter::offline_timeout_ms)
    {
        for (uint16_t i = 0; i < received.size; ++i)
        {
            remotes::reading value;
            if (remote.parser.push(received.data[i], value))
            {
                remote.latest.data = value;
                remote.latest.received_ms = received.received_ms;
                ++remote.latest.sequence;
                remote.latest.valid = true;
            }
        }
    }
    if (remote.started && bsp::usart::rx_status(remote.resource.instance) == status::ok &&
        fresh(remote.latest, now, user_config::remoter::offline_timeout_ms))
    {
        return;
    }
    if (uint32_t(now - remote.last_attempt_ms) < cfg::recovery_period_ms)
    {
        return;
    }
    remote.last_attempt_ms = now;
    remote.latest.valid = false;
    remote.parser.reset();
    if (bsp::usart::stop_rx(remote.resource.instance) != status::ok)
    {
        return;
    }
    {
        const auto mask = __get_PRIMASK();
        __disable_irq();
        remote.queued = {};
        __set_PRIMASK(mask);
    }
    remote.started = start_remote(remote);
    ++remote.restarts;
}

bool recover_can(can_bus& bus, bool motor, uint32_t now)
{
    auto* errors = bsp::can::err_sem(bus.resource.instance);
    bool error = bsp::can::current_status(bus.resource.instance) != status::ok;
    for (unsigned i = 0; errors != nullptr && i < 8; ++i)
    {
        if (tx_semaphore_get(errors, TX_NO_WAIT) != TX_SUCCESS)
        {
            break;
        }
        error = true;
    }
    if (error || !bus.ready)
    {
        const bool first_fault = bus.ready;
        bus.ready = false;
        const auto mask = __get_PRIMASK();
        __disable_irq();
        if (motor)
        {
            for (auto& received : motor_received)
            {
                received.valid = false;
            }
        }
        else
        {
            lower_received.valid = false;
        }
        __set_PRIMASK(mask);
        // Handle the first fault immediately, including non-bus-off notifications
        // where hardware can still hold a nonzero command. Throttle failed retries only.
        if (first_fault || uint32_t(now - bus.last_attempt_ms) >= cfg::recovery_period_ms)
        {
            bus.last_attempt_ms = now;
            bus.ready = bsp::can::restart(bus.resource.instance) == status::ok;
            if (motor)
            {
                ++diagnostic.motor_restarts;
            }
            else
            {
                ++diagnostic.lower_restarts;
            }
        }
        return true;
    }
    return false;
}

vision_telemetry make_telemetry(const imu::attitude::state& attitude, const control_input& input,
    const control_output& output)
{
    vision_telemetry result;
    result.detect_color = !input.lower.color;
    result.mode = 1;
    result.yaw = input.total_yaw * 180.0f / math::pi;//usb发出去的，所以角度
    result.pitch = input.pitch * 180.0f / math::pi;
    result.gyro_pitch = input.gyro_pitch;
    result.gyro_yaw = input.gyro_yaw;
    result.bullet_speed = input.lower.bullet_speed;
    result.bullet_count = output.bullet_count;
    // Rebuild the legacy gimbal quaternion after the old mounting-axis mapping.
    const float cy = std::cos(attitude.yaw * 0.5f);
    const float sy = std::sin(attitude.yaw * 0.5f);
    const float cp = std::cos(attitude.roll * 0.5f);
    const float sp = std::sin(attitude.roll * 0.5f);
    const float cr = std::cos(-attitude.pitch * 0.5f);
    const float sr = std::sin(-attitude.pitch * 0.5f);
    result.q[0] = cr * cp * cy + sr * sp * sy;
    result.q[1] = sr * cp * cy - cr * sp * sy;
    result.q[2] = cr * sp * cy + sr * cp * sy;
    result.q[3] = cr * cp * sy - sr * sp * cy;
    return result;
}

void control_entry(ULONG)
{
    controller control{control_settings()};
    imu::attitude::state attitude;
    sample<lower_status> lower;
    uint32_t lower_sequence = 0;
    uint32_t remote_sequences[2]{};
    for (auto& remote : remotes_rx)
    {
        if (remote.enabled)
        {
            remote.started = start_remote(remote);
            remote.last_attempt_ms = HAL_GetTick();
        }
    }
    ULONG deadline = tx_time_get();
    for (;;)
    {
        const auto started_ms = HAL_GetTick();
        const bool motor_recovering = recover_can(motor_bus, true, started_ms);
        const bool lower_recovering = recover_can(lower_bus, false, started_ms);
        sample<can_bytes> received;
        sample<vision_command> vision;
        {
            const auto mask = __get_PRIMASK();
            __disable_irq();
            received = lower_received;
            vision = vision_received;
            __set_PRIMASK(mask);
        }
        const bool motor_updates[]{motors.update_feedback(friction_left),
            motors.update_feedback(friction_right), motors.update_feedback(pitch)};
        for (unsigned i = 0; i < 3; ++i)
        {
            if (motor_updates[i] && !motor_recovering)
            {
                motor_received[i].received_ms = started_ms;
                ++motor_received[i].sequence;
                motor_received[i].valid = true;
            }
        }
        if (!received.valid)
        {
            lower.valid = false;
        }
        else if (received.sequence != lower_sequence)
        {
            lower_status value;
            if (decode_lower_status(received.data.bytes, 8, value))
            {
                lower = {value, received.received_ms, received.sequence, true};
            }
            lower_sequence = received.sequence;
        }
        for (auto& remote : remotes_rx)
        {
            read_remote(remote);
        }
        (void)attitude_reader.read(attitude);
        // Timestamp after the copies: an IRQ may have delivered a sample after
        // this cycle began, and unsigned age must not mistake it for an old one.
        const auto now = HAL_GetTick();
        control_input input;
        input.now_ms = now;
        // Preserve old input priority: an online DR16 takes precedence over VT03.
        for (unsigned i = 0; i < 2; ++i)
        {
            const auto& value = remotes_rx[i].latest;
            if (remotes_rx[i].enabled &&
                fresh(value, now, user_config::remoter::offline_timeout_ms))
            {
                input.remote = value.data;
                input.source = i == 0 ? remote_source::dr16 : remote_source::vt03;
                input.remote_valid = true;
                input.remote_new_frame = value.sequence != remote_sequences[i];
                remote_sequences[i] = value.sequence;
                break;
            }
        }
        input.pitch = attitude.roll;
        input.yaw = attitude.yaw;
        input.total_yaw = attitude.total_yaw;
        input.gyro_pitch = attitude.gyro.x;//pitch是x
        input.gyro_yaw = attitude.gyro.z;//yaw是z
        input.imu_valid = attitude.online && attitude.stage == imu::attitude::phase::running &&
                          uint32_t(now - attitude.received_ms) < cfg::imu_timeout_ms &&
                          std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) &&
                          std::isfinite(attitude.yaw) && std::isfinite(attitude.total_yaw) &&
                          std::isfinite(attitude.gyro.x) && std::isfinite(attitude.gyro.z);
        input.vision = vision.data;
        input.vision_valid = fresh(vision, now, cfg::vision_timeout_ms);
        input.lower = lower.data;
        input.lower_valid =
            lower_bus.ready && !lower_recovering && fresh(lower, now, cfg::lower_timeout_ms);
        input.friction_left_speed = friction_left.fdb.velocity;
        input.friction_right_speed = friction_right.fdb.velocity;
        input.friction_left_online =
            motor_bus.ready && !motor_recovering &&
            fresh(motor_received[0], now, cfg::motor_timeout_ms);
        input.friction_right_online =
            motor_bus.ready && !motor_recovering &&
            fresh(motor_received[1], now, cfg::motor_timeout_ms);
        input.pitch_online =
            motor_bus.ready && !motor_recovering &&
            fresh(motor_received[2], now, cfg::motor_timeout_ms);
        const auto output = control.update(input);
        yaw_debug.reference = output.yaw_reference;
        yaw_debug.feedback = input.total_yaw;
        yaw_debug.current = output.lower.yaw_cur;
        ++yaw_debug.sequence;

        pitch.motor_mode =
            output.mode == gimbal_mode::relax ? motor::mode::relax : motor::mode::current;
        pitch.cmd.current = output.pitch_current;
        if (std::abs(output.pitch_current) >= cfg::pitch_current_limit)
        {
            ++diagnostic.pitch_saturations;
        }
#ifdef smc_test_on_friction
        if (output.friction_enabled)
        {
            constexpr float dt = cfg::control_period_ms * 0.001f;
            const float left_current = friction_left_smc.update_velocity(
                output.friction_left_speed, friction_left.fdb.velocity, dt);
            const float right_current = friction_right_smc.update_velocity(
                output.friction_right_speed, friction_right.fdb.velocity, dt);
            if (friction_left_smc.state().valid && friction_right_smc.state().valid)
            {
                friction_left.motor_mode = motor::mode::current;
                friction_right.motor_mode = motor::mode::current;
                friction_left.cmd.current = static_cast<int16_t>(std::lround(left_current));
                friction_right.cmd.current = static_cast<int16_t>(std::lround(right_current));
            }
            else
            {
                friction_left_smc.reset();
                friction_right_smc.reset();
                friction_left.motor_mode = motor::mode::relax;
                friction_right.motor_mode = motor::mode::relax;
                friction_left.cmd.current = 0;
                friction_right.cmd.current = 0;
            }
        }
        else
        {
            friction_left_smc.reset();
            friction_right_smc.reset();
            friction_left.motor_mode = motor::mode::relax;
            friction_right.motor_mode = motor::mode::relax;
            friction_left.cmd.current = 0;
            friction_right.cmd.current = 0;
        }
#else
        friction_left.motor_mode = output.friction_enabled ? motor::mode::vel : motor::mode::relax;
        friction_right.motor_mode = friction_left.motor_mode;
        friction_left.cmd.velocity = output.friction_left_speed;
        friction_right.cmd.velocity = output.friction_right_speed;
        if (!output.friction_enabled)
        {
            friction_left.speed_pid.reset_state(0.0f, friction_left.fdb.velocity);
            friction_right.speed_pid.reset_state(0.0f, friction_right.fdb.velocity);
        }
#endif
        motors.update();
        if (motor_bus.ready && !motor_recovering)
        {
            motors.transmit(static_cast<uint32_t>(bsp::dwt::timeline_us()));
        }
        if (lower_bus.ready)
        {
            // Commands take priority over UI in FDCAN2's two-element TX FIFO.
            const auto command = encode_lower_command(output.lower);
            if (bsp::can::transmit(lower_bus.resource.instance, lower_command_id, command.data(),
                    command.size()) == status::ok)
            {
                const auto ui = encode_lower_ui(output.ui);
                if (bsp::can::transmit(lower_bus.resource.instance, lower_ui_id, ui.data(),
                        ui.size()) != status::ok)
                {
                    ++diagnostic.lower_tx_failures;
                }//
            }
            else
            {
                ++diagnostic.lower_tx_failures;
            }
        }

        // The legacy TX frame has no validity bits. Do not represent stale IMU or
        // lower-board color/ballistic data as a valid vision telemetry packet.
        sample<vision_telemetry> next;
        if (input.imu_valid && input.lower_valid)
        {
            next.data = make_telemetry(attitude, input, output);
            next.received_ms = attitude.received_ms;
            next.valid = true;
        }
        {
            const auto mask = __get_PRIMASK();
            __disable_irq();
            next.sequence = telemetry.sequence + 1;
            telemetry = next;
            __set_PRIMASK(mask);
        }
        deadline += cfg::control_period_ms;
        const auto remaining = static_cast<int32_t>(deadline - tx_time_get());
        if (remaining > 0)
        {
            tx_thread_sleep(static_cast<ULONG>(remaining));
        }
        else
        {
            ++diagnostic.control_overruns;
            tx_thread_sleep(1);
            deadline = tx_time_get();
        }
    }
}

void usb_entry(ULONG)
{
    for (;;)
    {
        sample<vision_telemetry> value;
        {
            const auto mask = __get_PRIMASK();
            __disable_irq();
            value = telemetry;
            __set_PRIMASK(mask);
        }
        if (bsp::usb::connected() && fresh(value, HAL_GetTick(), cfg::imu_timeout_ms))
        {
            const auto packet = encode_vision(value.data);
            uint32_t transferred = 0;
            const auto result = bsp::usb::write(packet.data(), packet.size(), transferred,
                cfg::usb_write_timeout_ms);
            if (result != status::ok || transferred != packet.size())
            {
                ++diagnostic.usb_tx_failures;
                if (transferred != 0 && transferred != packet.size())
                {
                    ++diagnostic.usb_partial_writes;
                }
            }
        }
        tx_thread_sleep(cfg::usb_period_ms);
    }
}
} // namespace
} // namespace big_head

extern "C" void app_start()
{
    using namespace big_head;
#ifdef smc_test_on_friction
    const auto smc_config = friction_smc_config();
    if (!friction_left_smc.configure(smc_config) ||
        !friction_right_smc.configure(smc_config))
    {
        Error_Handler();
        return;
    }
#else
    configure_friction_motor(friction_left);
    configure_friction_motor(friction_right);
#endif
    const auto motor_handle = motor_bus.resource.instance;
    if (bsp::dwt::init() != status::ok ||
        !motors.register_motor(friction_left, motor_handle, cfg::friction_left_id) ||
        !motors.register_motor(friction_right, motor_handle, cfg::friction_right_id) ||
        !motors.register_motor(pitch, motor_handle, cfg::pitch_id))
    {
        Error_Handler();
        return;
    }
    motors.set_period(cfg::control_period_ms * 0.001f);
    if (!motors.set_tx_period(motor_handle, 0x200, cfg::control_period_ms * 1000) ||
        !motors.set_tx_period(motor_handle, 0x1FF, cfg::control_period_ms * 1000) ||
        bsp::can::register_rx_callback(lower_bus.resource.instance,
            bsp::can::rx_callback::bind<can_receive>()) != status::ok)
    {
        Error_Handler();
        return;
    }
    for (auto* bus : {&motor_bus, &lower_bus})
    {
        if (bsp::can::init(bus->resource) != status::ok)
        {
            Error_Handler();
            return;
        }
        bus->ready = true;
        bus->last_attempt_ms = HAL_GetTick() - cfg::recovery_period_ms;
    }
    if (bsp::usb::on_rx(bsp::usb::rx_callback::bind<usb_receive>()) != status::ok ||
        imu::start_bmi088() != status::ok)
    {
        Error_Handler();
        return;
    }
    attitude_reader = imu::attitude::output.subscribe();
    if (!attitude_reader ||
        tx_thread_create(&control_thread, const_cast<CHAR*>("big_head"), control_entry, 0,
            control_stack, sizeof(control_stack), cfg::control_priority, cfg::control_priority,
            TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS ||
        tx_thread_create(&usb_thread, const_cast<CHAR*>("vision_tx"), usb_entry, 0, usb_stack,
            sizeof(usb_stack), cfg::usb_priority, cfg::usb_priority, TX_NO_TIME_SLICE,
            TX_AUTO_START) != TX_SUCCESS)
    {
        Error_Handler();
    }
}
