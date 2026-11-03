#include "protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace
{

constexpr std::array<uint8_t, 28> vision_frame{0xA5, 0x02, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00,
    0xC0, 0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0xBE, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x80,
    0xC0, 0x0E, 0x62};

void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::fprintf(stderr, "protocol test failed: %s\n", message);
        std::exit(EXIT_FAILURE);
    }
}

// Independent bitwise reference for constructing corrupted/nonfinite stream test cases.
void checksum(std::array<uint8_t, 28>& bytes)
{
    uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < bytes.size() - 2; ++i)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
        {
            crc = static_cast<uint16_t>((crc >> 1U) ^ ((crc & 1U) != 0 ? 0x8408U : 0U));
        }
    }
    bytes[26] = static_cast<uint8_t>(crc);
    bytes[27] = static_cast<uint8_t>(crc >> 8U);
}

std::size_t feed(big_head::vision_parser& parser, const uint8_t* data, std::size_t size,
    big_head::vision_command& command)
{
    std::size_t accepted = 0;
    for (std::size_t i = 0; i < size; ++i)
    {
        if (parser.push(data[i], command))
        {
            ++accepted;
        }
    }
    return accepted;
}

void check_command(const big_head::vision_command& command)
{
    require(command.mode == 2 && command.yaw == 1.0f && command.yaw_vel == -2.0f &&
                command.yaw_acc == 0.5f && command.pitch == -0.25f && command.pitch_vel == 3.0f &&
                command.pitch_acc == -4.0f,
        "vision RX golden values and order");
}

void test_vision_stream()
{
    for (std::size_t split = 0; split <= vision_frame.size(); ++split)
    {
        big_head::vision_parser parser;
        big_head::vision_command command{};
        const auto first = feed(parser, vision_frame.data(), split, command);
        require(first == (split == vision_frame.size() ? 1U : 0U), "no premature frame");
        require(first + feed(parser, vision_frame.data() + split, vision_frame.size() - split,
                            command) ==
                    1,
            "accept every USB fragmentation boundary");
        check_command(command);
    }

    big_head::vision_parser parser;
    big_head::vision_command command{};
    std::vector<uint8_t> stream{0x00, 0x5A, 0xA5, 0x00, 0xA5, 0xAA, 0xBB};
    for (unsigned i = 0; i < 3; ++i)
    {
        stream.insert(stream.end(), vision_frame.begin(), vision_frame.end());
    }
    require(feed(parser, stream.data(), stream.size(), command) == 3,
        "resynchronize nested false headers then accept coalesced frames");
    check_command(command);

    auto damaged = vision_frame;
    damaged[10] ^= 0x01;
    require(feed(parser, damaged.data(), damaged.size(), command) == 0, "reject bad CRC");
    check_command(command);
    require(feed(parser, vision_frame.data(), vision_frame.size(), command) == 1,
        "recover after bad CRC");

    parser.reset();
    require(feed(parser, vision_frame.data(), 12, command) == 0, "buffer a partial frame");
    parser.reset();
    require(feed(parser, vision_frame.data() + 12, vision_frame.size() - 12, command) == 0,
        "reset discards partial frame");
    require(feed(parser, vision_frame.data(), vision_frame.size(), command) == 1,
        "accept complete frame after reset");

    auto embedded_header = vision_frame;
    embedded_header[2] = 0xA5;
    checksum(embedded_header);
    require(feed(parser, embedded_header.data(), embedded_header.size(), command) == 1,
        "header byte inside a valid payload is data");

    for (std::size_t dropped = 0; dropped < vision_frame.size(); ++dropped)
    {
        parser.reset();
        stream.assign(vision_frame.begin(), vision_frame.end());
        stream.erase(stream.begin() + static_cast<std::ptrdiff_t>(dropped));
        stream.insert(stream.end(), vision_frame.begin(), vision_frame.end());
        require(feed(parser, stream.data(), stream.size(), command) == 1,
            "recover a complete frame after any single-byte loss");
        check_command(command);
    }
}

void test_nonfinite()
{
    big_head::vision_parser parser;
    big_head::vision_command command{};
    for (std::size_t offset = 2; offset <= 22; offset += 4)
    {
        for (uint32_t bits : {0x7F800000U, 0xFF800000U, 0x7FC00000U, 0x7F800001U})
        {
            auto invalid = vision_frame;
            for (unsigned byte = 0; byte < 4; ++byte)
            {
                invalid[offset + byte] = static_cast<uint8_t>(bits >> (byte * 8U));
            }
            checksum(invalid);
            command.mode = 0xEE;
            require(feed(parser, invalid.data(), invalid.size(), command) == 0,
                "reject NaN and either infinity in all six control fields");
            require(command.mode == 0xEE, "invalid data leaves last accepted value unchanged");
            require(feed(parser, vision_frame.data(), vision_frame.size(), command) == 1,
                "recover immediately after nonfinite data");
        }
    }
}

void test_vision_tx()
{
    const big_head::vision_telemetry telemetry{true, true, 1, {1.0f, 0.5f, -0.5f, 0.0f}, 180.0f,
        -2.0f, -30.0f, 3.0f, 22.5f, 0x1234};
    constexpr std::array<uint8_t, 44> expected{0x5A, 0x03, 0x01, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00,
        0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0xBF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x43,
        0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0xF0, 0xC1, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0xB4,
        0x41, 0x34, 0x12, 0xCC, 0xCE};
    require(big_head::encode_vision(telemetry) == expected,
        "vision TX golden bytes, reserved byte, quaternion order, CRC and bullet count");
}

void test_lower_status()
{
    constexpr std::array<uint8_t, 8> frame{0x2B, 0xCB, 0x80, 0x00, 0x00, 0xB4, 0x41, 0x96};
    big_head::lower_status status{};
    require(big_head::decode_lower_status(frame.data(), frame.size(), status),
        "accept C1 golden frame");
    require(status.inited && status.color && status.level == 10 && status.heatlimit == 300 &&
                status.heatnow == 257 && status.bullet_speed == 22.5f,
        "C1 cross-byte bitfields and unaligned float");

    for (unsigned bit = 0; bit < 24; ++bit)
    {
        std::array<uint8_t, 8> one_bit{};
        one_bit[bit / 8] = static_cast<uint8_t>(1U << (bit % 8));
        require(big_head::decode_lower_status(one_bit.data(), one_bit.size(), status),
            "accept each isolated C1 status bit");
        require(status.inited == (bit == 0) && status.color == (bit == 1), "C1 flag bits");
        require(status.level == (bit >= 2 && bit < 6 ? 1U << (bit - 2) : 0U), "C1 level bits");
        require(status.heatlimit == (bit >= 6 && bit < 15 ? 1U << (bit - 6) : 0U),
            "C1 heat limit bits");
        require(status.heatnow == (bit >= 15 ? 1U << (bit - 15) : 0U), "C1 current heat bits");
    }
    require(!big_head::decode_lower_status(nullptr, 8, status), "reject null C1 data");
    require(!big_head::decode_lower_status(frame.data(), 7, status), "reject short C1 data");
    require(!big_head::decode_lower_status(frame.data(), 9, status), "reject oversized C1 data");
    auto invalid = frame;
    invalid[3] = 0x00;
    invalid[4] = 0x00;
    invalid[5] = 0x80;
    invalid[6] = 0x7F;
    status.level = 11;
    require(!big_head::decode_lower_status(invalid.data(), invalid.size(), status),
        "reject nonfinite C1 bullet speed");
    require(status.level == 11, "invalid C1 leaves accepted status unchanged");
}

void test_lower_tx()
{
    big_head::lower_command command{true, false, true, false, true, false, true, -12345, -128, 127,
        -1, -8};
    constexpr std::array<uint8_t, 8> expected{0x55, 0xC7, 0xCF, 0x80, 0x7F, 0xFF, 0xF8, 0x00};
    require(big_head::encode_lower_command(command) == expected,
        "B2 golden signed current/speeds and zero reserved byte");
    command = {false, true, false, true, false, true, false, -32768, 0, 0, 0, 0};
    constexpr std::array<uint8_t, 8> complement{0x2A, 0x00, 0x80, 0, 0, 0, 0, 0};
    require(big_head::encode_lower_command(command) == complement,
        "B2 complementary flags and most negative current");
    require(big_head::encode_lower_command({}) == std::array<uint8_t, 8>{}, "B2 safe zero frame");

    const big_head::lower_ui ui{true, false, true, false, 0xFA, 0xCC, 0x42};
    constexpr std::array<uint8_t, 8> expected_ui{0xA5, 0xCC, 0x42, 0, 0, 0, 0, 0};
    require(big_head::encode_lower_ui(ui) == expected_ui, "B1 flags, four-bit target and padding");
    const big_head::lower_ui complement_ui{false, true, false, true, 0, 0, 0};
    constexpr std::array<uint8_t, 8> expected_complement_ui{0x0A, 0, 0, 0, 0, 0, 0, 0};
    require(big_head::encode_lower_ui(complement_ui) == expected_complement_ui,
        "B1 complementary flags");
}

} // namespace

int main()
{
    test_vision_stream();
    test_nonfinite();
    test_vision_tx();
    test_lower_status();
    test_lower_tx();
    std::puts("big_head protocol tests passed");
}
