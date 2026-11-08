#include "remote_stream.hpp"
#include "crc.hpp"
#include "dji_manager.hpp"
#include "gm6020.hpp"
#include "m3508.hpp"
#include <array>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>

namespace test_can
{
motor::dji::frame_t frames[4];
unsigned count = 0;
bsp::can::handle registered_bus = bsp::can::handle::none;
bsp::can::rx_callback callback;
uint16_t failing_id = 0;
}

namespace bsp::can
{
pnxtypes::status register_rx_callback(handle instance, rx_callback next) noexcept
{
    if (test_can::callback || !next)
    {
        return pnxtypes::status::busy;
    }
    test_can::registered_bus = instance;
    test_can::callback = next;
    return pnxtypes::status::ok;
}

pnxtypes::status transmit(handle instance, uint32_t id, const uint8_t* data, uint16_t len) noexcept
{
    if (instance != test_can::registered_bus || !data || len != 8 ||
        test_can::count == std::size(test_can::frames))
    {
        return pnxtypes::status::invalid_arg;
    }
    if (id == test_can::failing_id)
    {
        return pnxtypes::status::busy;
    }
    auto& frame = test_can::frames[test_can::count++];
    frame.id = static_cast<uint16_t>(id);
    frame.length = static_cast<uint8_t>(len);
    std::memcpy(frame.data, data, len);
    return pnxtypes::status::ok;
}
} // namespace bsp::can

namespace
{
void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::fprintf(stderr, "remote/motor test failed: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

void bits(uint8_t* bytes, unsigned start, unsigned count, uint32_t value)
{
    for (unsigned i = 0; i < count; ++i)
    {
        if ((value & (1U << i)) != 0)
        {
            bytes[(start + i) / 8] |= static_cast<uint8_t>(1U << ((start + i) % 8));
        }
    }
}

std::array<uint8_t, 21> vt03_frame()
{
    std::array<uint8_t, 21> bytes{};
    bytes[0] = 0xA9;
    bytes[1] = 0x53;
    bits(bytes.data() + 2, 0, 11, 1684);
    bits(bytes.data() + 2, 11, 11, 364);
    bits(bytes.data() + 2, 22, 11, 1024);
    bits(bytes.data() + 2, 33, 11, 1354);
    bits(bytes.data() + 2, 44, 2, 1);
    bits(bytes.data() + 2, 48, 1, 1);
    bits(bytes.data() + 2, 49, 11, 1024);
    bytes[10] = 7;
    bytes[16] = 1;
    crc::append_crc16_checksum(bytes.data(), bytes.size());
    return bytes;
}

void test_uart_streams()
{
    const auto frame = vt03_frame();
    for (std::size_t split = 0; split <= frame.size(); ++split)
    {
        big_head::remote_stream parser{true};
        remotes::reading output;
        unsigned frames = 0;
        for (std::size_t i = 0; i < split; ++i)
        {
            frames += parser.push(frame[i], output);
        }
        for (std::size_t i = split; i < frame.size(); ++i)
        {
            frames += parser.push(frame[i], output);
        }
        require(frames == 1 && output.fn_2 && output.left_sw == remotes::sw_state::mid &&
                    output.right_x == 1 && output.right_y == -1 && output.left_x == 0.5f &&
                    output.mouse_x == 7 && output.mouse_left,
            "VT03 frame split preserves normalized controls and byte layout");
    }
    big_head::remote_stream parser{true};
    remotes::reading output;
    unsigned frames = 0;
    (void)parser.push(0x00, output);
    auto broken = frame;
    broken[20] ^= 1;
    for (auto byte : broken)
    {
        frames += parser.push(byte, output);
    }
    for (unsigned repeat = 0; repeat < 2; ++repeat)
    {
        for (auto byte : frame)
        {
            frames += parser.push(byte, output);
        }
    }
    require(frames == 2, "VT03 bad CRC/junk resynchronizes before concatenated frames");
    for (unsigned i = 0; i < 10; ++i)
    {
        (void)parser.push(frame[i], output);
    }
    parser.reset();
    frames = 0;
    for (auto byte : frame)
    {
        frames += parser.push(byte, output);
    }
    require(frames == 1, "UART restart discards partial frame");

    std::array<uint8_t, 18> dr16{};
    for (unsigned i = 0; i < 4; ++i)
    {
        bits(dr16.data(), i * 11, 11, 1024);
    }
    bits(dr16.data(), 44, 2, 1); // right up: fire
    bits(dr16.data(), 46, 2, 3); // left middle: normal
    bits(dr16.data(), 128, 11, 1024);
    big_head::remote_stream dr_parser{false};
    frames = 0;
    for (auto byte : dr16)
    {
        frames += dr_parser.push(byte, output);
    }
    require(frames == 1 && output.left_sw == remotes::sw_state::mid &&
                output.right_sw == remotes::sw_state::up && output.right_x == 0,
        "DR16 switch normalization matches old control modes");
}

void test_motor_model()
{
    motor::dji::m3508 left{motor::dji::m3508::gear_ratio::direct_drive};
    motor::dji::m3508 right{motor::dji::m3508::gear_ratio::direct_drive};
    motor::dji::m3508 standard;
    motor::dji::m3508 xroll{motor::dji::m3508::gear_ratio::xroll_gearbox};
    motor::dji::gm6020 pitch;
    motor::dji::manager motors;
    const auto bus = bsp::can::handle::fdcan1;
    const float direct_ratio = left.configuration().gear_ratio;
    const float standard_ratio = standard.configuration().gear_ratio;
    const float xroll_ratio = xroll.configuration().gear_ratio;
    require(std::fabs(direct_ratio - 1.0f) < 0.0001f &&
                std::fabs(standard_ratio - 3591.0f / 187.0f) < 0.0001f &&
                std::fabs(xroll_ratio - 268.0f / 17.0f) < 0.0001f,
        "M3508 exposes direct, standard, and XRoll gear ratios");
    require(motors.register_motor(left, bus, 0x201) &&
                motors.register_motor(right, bus, 0x202) &&
                motors.register_motor(pitch, bus, 0x205) &&
                motors.set_tx_period(bus, 0x200, 1000) &&
                motors.set_tx_period(bus, 0x1FF, 1000),
        "local motor IDs register on the configured BSP CAN bus");
    require(test_can::registered_bus == bus && test_can::callback,
        "manager registers one CAN receive callback before initialization");
    const uint8_t feedback[8]{0, 0, 0x16, 0x62, 0, 0, 40, 0}; // 5730 rotor rpm
    bsp::can::rx_frame received;
    received.id = 0x201;
    received.len = 8;
    std::memcpy(received.data, feedback, sizeof(feedback));
    test_can::callback(bus, received);
    require(motors.update_feedback(left) && !motors.update_feedback(left),
        "control task consumes each buffered feedback frame once");
    require(std::fabs(left.fdb.velocity - 600.0442f) < 0.01f,
        "friction model uses rotor rad/s without the stock 19.2 gearbox");
    const auto valid_position = left.fdb.position;
    const auto valid_velocity = left.fdb.velocity;
    const auto valid_age = left.feedback_age();
    received.data[0] = 0x20;
    received.data[1] = 0x00;
    test_can::callback(bus, received);
    require(!motors.update_feedback(left) && left.fdb.position == valid_position &&
                left.fdb.velocity == valid_velocity && left.feedback_age() == valid_age,
        "feedback with an encoder value outside its resolution is rejected");
    left.motor_mode = motor::mode::current;
    left.cmd.current = 1234;
    right.motor_mode = motor::mode::current;
    right.cmd.current = -1234;
    pitch.motor_mode = motor::mode::current;
    pitch.cmd.current = 25000;
    motors.update();
    motors.transmit(0);
    require(test_can::count == 2 && test_can::frames[0].id == 0x200 &&
                test_can::frames[1].id == 0x1FF,
        "manager sends both legacy command groups");
    require(test_can::frames[0].data[0] == 0x04 && test_can::frames[0].data[1] == 0xD2 &&
                test_can::frames[0].data[2] == 0xFB && test_can::frames[0].data[3] == 0x2E,
        "friction current signs preserved in big-endian slots");
    require(test_can::frames[1].data[0] == 0x3A && test_can::frames[1].data[1] == 0x98,
        "pitch output clamped to 15000 by existing driver");
    test_can::count = 0;
    test_can::failing_id = 0x1FF;
    motors.transmit(1000);
    require(test_can::count == 1 && test_can::frames[0].id == 0x200,
        "a busy CAN group does not block the other command group");
    test_can::count = 0;
    test_can::failing_id = 0;
    left.motor_mode = motor::mode::relax;
    right.motor_mode = motor::mode::relax;
    pitch.motor_mode = motor::mode::relax;
    motors.update();
    motors.transmit(2000);
    require(test_can::count == 2 && test_can::frames[0].id == 0x200 &&
                test_can::frames[1].id == 0x1FF,
        "a failed command group is retried on the next control tick");
    for (unsigned i = 0; i < test_can::count; ++i)
    {
        const auto& frame = test_can::frames[i];
        for (auto byte : frame.data)
        {
            require(byte == 0, "relax sends zero current for every local motor");
        }
    }
}
} // namespace

int main()
{
    test_uart_streams();
    test_motor_model();
    std::puts("Remote framing and motor model tests passed");
}
