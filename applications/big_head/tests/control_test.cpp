#include "control.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace
{
using namespace big_head;

void require(bool value, const char* message)
{
    if (!value)
    {
        std::fprintf(stderr, "control test failed: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

bool near(float a, float b)
{
    return std::fabs(a - b) < 0.0001f;
}

// A fixed legacy tuning fixture, independent of hardware/generated config.
control_parameters settings()
{
    return {{140, 0, 3000, 500, 10}, {4000, 0, 10000, 25000, 100}, {80, 0, 800, 500, 50},
        {200, 0, 0, 500, 50}, {0.009f, 0.02f, 0.6151134f, 1.35f}, {0, 0.02f, 0.6151134f, 1.35f},
        {0.1f, 0.543f}, {0.001f, 0.543f}, {0.001f, 0.543f}, 25000, 15000, 50, 1750, 2000, -0.6f,
        0.4f, 0.005f, 0.05f, 0.025f, 0.01f, 0.002f, 600, 0.9f, 6, 0.7f, 0.01f, 4, 8, 16, -8, 1000,
        true, true, true};
}

control_input ready_input()
{
    control_input input;
    input.source = remote_source::vt03;
    input.remote_valid = true;
    input.remote_new_frame = true;
    input.imu_valid = true;
    input.pitch_online = true;
    input.friction_left_online = true;
    input.friction_right_online = true;
    input.friction_left_speed = 600;
    input.friction_right_speed = -600;
    input.lower_valid = true;
    input.lower.inited = true;
    input.lower.heatlimit = 100;
    return input;
}

void safe(const control_output& output, const char* message)
{
    require(output.pitch_current == 0 && !output.friction_enabled && output.lower.yaw_cur == 0 &&
                output.lower.tri_spd == 0 && !output.lower.ifmove &&
                output.mode == gimbal_mode::relax,
        message);
}

void arm(controller& control, control_input& input)
{
    input.remote.left_sw = remotes::sw_state::low;
    input.remote_new_frame = true;
    safe(control.update(input), "low switch is relaxed");
    input.remote.left_sw = remotes::sw_state::mid;
    const auto output = control.update(input);
    require(output.armed && output.mode == gimbal_mode::manual, "low to mid arms manual control");
}

void warm(controller& control, control_input& input)
{
    input.remote.fn_2 = true;
    input.remote_new_frame = true;
    require(control.update(input).friction_enabled, "Fn2 first edge starts friction wheels");
    input.remote.fn_2 = false;
    (void)control.update(input);
}

void test_remote_and_faults()
{
    controller control{settings()};
    auto input = ready_input();
    input.remote.left_sw = remotes::sw_state::mid;
    safe(control.update(input), "startup mid does not arm");
    safe(control.update(input), "held startup mid still does not arm");
    arm(control, input);
    warm(control, input);
    input.remote.mouse_left = true;
    auto output = control.update(input);
    require(output.lower.tri_spd == 8 && output.friction_left_speed == 600 &&
                output.friction_right_speed == -600,
        "warm left-click fires with legacy wheel signs");
    input.remote.key.CTRL = true;
    require(control.update(input).lower.tri_spd == -8, "Ctrl left-click reverses feeder");
    input.remote.key.CTRL = false;

    input.lower.heatnow = 70;
    output = control.update(input);
    require(output.lower.tri_spd == 0 && output.friction_enabled,
        "70 percent heat stops feed only");
    input.lower.heatnow = 0;
    input.lower.heatlimit = 0;
    require(control.update(input).lower.tri_spd == 0, "unknown heat limit inhibits feed");
    input.lower.heatlimit = 100;
    input.friction_left_speed = 539;
    require(control.update(input).lower.tri_spd == 0, "feed waits for wheel speed");
    input.friction_left_speed = -600;
    require(control.update(input).lower.tri_spd == 0, "wrong wheel direction inhibits feed");
    input.friction_left_speed = 600;
    input.friction_right_online = false;
    output = control.update(input);
    require(!output.friction_enabled && output.lower.tri_spd == 0, "one offline wheel stops pair");
    input.friction_right_online = true;
    input.lower_valid = false;
    output = control.update(input);
    require(!output.friction_enabled && output.lower.tri_spd == 0 && output.lower.yaw_cur == 0 &&
                !output.lower.ifmove,
        "lower loss closes lower outputs");
    input.lower_valid = true;
    input.imu_valid = false;
    safe(control.update(input), "IMU loss zeroes all outputs");
    input.imu_valid = true;
    input.pitch_online = false;
    safe(control.update(input), "pitch feedback loss zeroes all outputs");
    input.pitch_online = true;
    input.remote_valid = false;
    safe(control.update(input), "remote loss zeroes all outputs");
    input.remote_valid = true;
    safe(control.update(input), "reconnect requires rearming");
    safe(control.update(input), "held mode after reconnect remains safe");
    input.remote.key.CTRL = true;
    input.remote.key.X = true;
    require(control.update(input).armed, "explicit Ctrl X rising rearms");
    input.remote.key.Z = true;
    safe(control.update(input), "Ctrl Z disarms");
    input.remote.key = {};
    arm(control, input);
    input.remote.pause = true;
    safe(control.update(input), "VT03 pause stops immediately");
    input.remote.pause = false;
    safe(control.update(input), "pause release alone does not arm");
}

void test_edges_and_takeover()
{
    controller control{settings()};
    auto input = ready_input();
    arm(control, input);
    input.remote.fn_2 = true;
    require(control.update(input).friction_enabled, "Fn2 warms");
    input.remote_new_frame = false;
    require(control.update(input).lower.tri_spd == 0, "cached Fn2 is not another edge");
    input.remote_new_frame = true;
    require(control.update(input).lower.tri_spd == 0, "held Fn2 is not another edge");
    input.remote.fn_2 = false;
    (void)control.update(input);
    input.remote.fn_2 = true;
    require(control.update(input).lower.tri_spd == 8, "second Fn2 enters fire");
    input.remote.fn_1 = true;
    require(!control.update(input).friction_enabled, "Fn1 closes shooter");
    input.source = remote_source::dr16;
    input.remote.right_sw = remotes::sw_state::up;
    safe(control.update(input), "DR16 takeover does not inherit armed state");
    arm(control, input);
    require(control.update(input).friction_enabled, "DR16 upper shoot switch enables wheels");
    input.remote.right_sw = remotes::sw_state::low;
    require(!control.update(input).friction_enabled, "DR16 lower shoot switch closes wheels");

    input.remote.key.R = true;
    const auto turn = control.update(input);
    require(turn.lower.ifturn && near(turn.yaw_reference, 3.14159265f), "turn edge sets pi target");
    input.remote_new_frame = false;
    const auto held = control.update(input);
    require(!held.lower.ifturn && near(held.yaw_reference, turn.yaw_reference),
        "turn command is a pulse but yaw target persists");
}

void test_aim_and_limits()
{
    controller control{settings()};
    auto input = ready_input();
    arm(control, input);
    input.pitch = -0.59f;
    input.remote.right_y = 1;
    auto output = control.update(input);
    require(output.pitch_reference >= -0.6f && output.pitch_current >= -15000 &&
                output.pitch_current <= 15000,
        "manual pitch obeys angle/current limits");
    input.remote.right_y = 0;
    input.pitch = 0;
    input.vision_valid = true;
    input.vision.mode = 1;
    input.vision.pitch = -1;
    output = control.update(input);
    require(output.mode == gimbal_mode::automatic && near(output.pitch_reference, 0.4f),
        "auto pitch obeys upper limit and old sign convention");
    input.vision_valid = false;
    output = control.update(input);
    require(output.mode == gimbal_mode::manual && near(output.pitch_reference, input.pitch),
        "stale vision returns to current manual attitude in same cycle");
    input.vision_valid = true;
    input.yaw = 3.13f;
    input.total_yaw = 3.13f;
    input.vision.yaw = -3.13f;
    input.vision.pitch = 0;
    output = control.update(input);
    require(output.mode == gimbal_mode::automatic && output.yaw_reference > 3.13f &&
                output.yaw_reference < 3.2f,
        "auto yaw unwraps across pi");
    input.vision.yaw = 0;
    require(control.update(input).mode == gimbal_mode::manual, "reject over 90 degree target jump");
    input.yaw = 0;
    input.total_yaw = 0;
    input.vision.pitch = std::numeric_limits<float>::quiet_NaN();
    require(control.update(input).mode == gimbal_mode::manual, "NaN target cannot enable auto");
    input.pitch = std::numeric_limits<float>::infinity();
    safe(control.update(input), "nonfinite attitude closes outputs");
}

void test_automatic_shot()
{
    controller control{settings()};
    auto input = ready_input();
    arm(control, input);
    warm(control, input);
    input.vision_valid = true;
    input.vision.mode = 2;
    input.now_ms = 0xFFFFFFF0U;
    auto output = control.update(input);
    require(output.auto_fire && output.lower.tri_spd == 8, "aligned mode 2 starts auto shot");
    input.friction_left_speed = 593;
    input.now_ms += 1;
    output = control.update(input);
    require(!output.auto_fire && output.lower.tri_spd == 0 && output.bullet_count == 1,
        "wheel drop ends shot and counts once");
    input.friction_left_speed = 600;
    input.now_ms += 1;
    require(control.update(input).auto_fire, "valid next auto request can start another shot");
    input.now_ms += 1000;
    output = control.update(input);
    require(!output.auto_fire && output.lower.tri_spd == 0,
        "auto shot timeout works over tick wrap");
    require(control.update(input).lower.tri_spd == 0, "timeout stays latched for held request");
    input.vision.mode = 1;
    require(control.update(input).lower.tri_spd == 0, "request release resets timeout latch");
    input.vision.mode = 2;
    require(control.update(input).auto_fire, "new request after release can fire");
    input.vision_valid = false;
    output = control.update(input);
    require(!output.auto_fire && output.lower.tri_spd == 0, "target loss cancels in-flight feed");

    input.vision_valid = true;
    input.pitch = -0.6f;
    input.vision.pitch = 1.0f;
    output = control.update(input);
    require(near(output.pitch_reference, -0.6f) && output.lower.tri_spd == 0,
        "a target beyond mechanical pitch travel must not count as aligned");
}

void test_manual_takeover()
{
    controller control{settings()};
    auto input = ready_input();
    arm(control, input);
    input.remote.mouse_x = 100;
    input.remote.mouse_y = 100;
    (void)control.update(input);
    input.vision_valid = true;
    (void)control.update(input);
    input.remote.mouse_x = 0;
    input.remote.mouse_y = 0;
    for (unsigned i = 0; i < 20; ++i)
    {
        (void)control.update(input);
    }
    input.vision_valid = false;
    const auto output = control.update(input);
    require(output.mode == gimbal_mode::manual && near(output.yaw_reference, input.total_yaw) &&
                near(output.pitch_reference, input.pitch),
        "manual takeover does not replay displacement from before AUTO");
}
} // namespace

int main()
{
    test_remote_and_faults();
    test_edges_and_takeover();
    test_aim_and_limits();
    test_automatic_shot();
    test_manual_takeover();
    std::puts("Control state, safety, limits and shot tests passed");
}
