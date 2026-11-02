#include "remote_stream.hpp"
#include "crc.hpp"
#include "dji_manager.hpp"
#include "gm6020.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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

class friction : public motor::dji::motor
{
public:
    friction() : motor({1, 15000, 15000, 100, 8192})
    {
    }
};

struct transmitted
{
    motor::dji::frame_t frames[2];
    unsigned count = 0;
};

bool send(void* context, uint8_t bus, const motor::dji::frame_t& frame)
{
    auto& sent = *static_cast<transmitted*>(context);
    require(bus == 0 && sent.count < 2, "DJI uses one configured logical bus");
    sent.frames[sent.count++] = frame;
    return true;
}

void test_motor_model()
{
    friction left;
    friction right;
    motor::dji::gm6020 pitch;
    transmitted sent{};
    motor::dji::manager motors{send, &sent};
    require(motors.register_motor(left, 0, 0x201) && motors.register_motor(right, 0, 0x202) &&
                motors.register_motor(pitch, 0, 0x205),
        "legacy local motor IDs register");
    const uint8_t feedback[8]{0, 0, 0x16, 0x62, 0, 0, 40, 0}; // 5730 rotor rpm
    motors.receive(0, 0x201, feedback);
    require(std::fabs(left.fdb.velocity - 600.0442f) < 0.01f,
        "friction model uses rotor rad/s without the stock 19.2 gearbox");
    left.motor_mode = motor::mode::current;
    left.cmd.current = 1234;
    right.motor_mode = motor::mode::current;
    right.cmd.current = -1234;
    pitch.motor_mode = motor::mode::current;
    pitch.cmd.current = 25000;
    motors.update();
    motors.transmit(0);
    require(sent.count == 2 && sent.frames[0].id == 0x200 && sent.frames[1].id == 0x1FF,
        "manager sends both legacy command groups");
    require(sent.frames[0].data[0] == 0x04 && sent.frames[0].data[1] == 0xD2 &&
                sent.frames[0].data[2] == 0xFB && sent.frames[0].data[3] == 0x2E,
        "friction current signs preserved in big-endian slots");
    require(sent.frames[1].data[0] == 0x3A && sent.frames[1].data[1] == 0x98,
        "pitch output clamped to 15000 by existing driver");
    sent.count = 0;
    left.motor_mode = motor::mode::relax;
    right.motor_mode = motor::mode::relax;
    pitch.motor_mode = motor::mode::relax;
    motors.update();
    motors.transmit(1000);
    for (const auto& frame : sent.frames)
    {
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
