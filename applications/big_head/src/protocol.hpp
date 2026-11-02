#pragma once

#include "crc.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace big_head
{

inline constexpr uint32_t lower_status_id = 0xC1;
inline constexpr uint32_t lower_command_id = 0xB2;
inline constexpr uint32_t lower_ui_id = 0xB1;
inline constexpr std::size_t vision_command_size = 28;
inline constexpr std::size_t vision_telemetry_size = 44;

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
    "The legacy wire protocol requires IEEE 754 binary32 floats");

struct vision_command
{
    uint8_t mode{};
    // Angles in rad, velocities in rad/s, accelerations in rad/s^2.
    float yaw{};
    float yaw_vel{};
    float yaw_acc{};
    float pitch{};
    float pitch_vel{};
    float pitch_acc{};
};

struct vision_telemetry
{
    bool detect_color{};
    bool reset_tracker{};
    uint8_t mode{};
    float q[4]{1.0f, 0.0f, 0.0f, 0.0f}; // W, X, Y, Z in the legacy gimbal frame.
    float yaw{};                        // Degrees; continuous yaw.
    float gyro_yaw{};                   // rad/s.
    float pitch{};                      // Degrees.
    float gyro_pitch{};                 // rad/s.
    float bullet_speed{};               // m/s.
    uint16_t bullet_count{};
};

struct lower_status
{
    bool inited{};
    bool color{};
    uint8_t level{};
    uint16_t heatlimit{};
    uint16_t heatnow{};
    float bullet_speed{};
};

struct lower_command
{
    bool ifmove{};
    bool ifjump{};
    bool ifspin{};
    bool ifturn{};
    bool ifstair{};
    bool iffly{};
    bool ifgimbalonly{};
    int16_t yaw_cur{};
    int8_t vx{};
    int8_t vy{};
    int8_t len_level{};
    int8_t tri_spd{};
};

struct lower_ui
{
    bool reset{};
    bool fire{};
    bool bulletfreq{};
    bool aim_rune{};
    uint8_t aim_target_now{};
    uint8_t aim_target_x{};
    uint8_t aim_target_y{};
};

inline uint32_t read_u32_le(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
           (static_cast<uint32_t>(data[2]) << 16U) | (static_cast<uint32_t>(data[3]) << 24U);
}

inline float read_float_le(const uint8_t* data)
{
    const uint32_t bits = read_u32_le(data);
    float value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

inline bool finite_float_le(const uint8_t* data)
{
    // Inspect the wire representation so validation also survives fast-math compiler options.
    return (read_u32_le(data) & 0x7F800000U) != 0x7F800000U;
}

inline void write_float_le(uint8_t* data, float value)
{
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (unsigned i = 0; i < sizeof(bits); ++i)
    {
        data[i] = static_cast<uint8_t>(bits >> (i * 8U));
    }
}

// Own one parser in the USB RX thread. Chunk boundaries do not affect frame detection.
class vision_parser
{
public:
    bool push(uint8_t byte, vision_command& command)
    {
        if (size_ == 0 && byte != 0xA5)
        {
            return false;
        }

        frame_[size_++] = byte;
        if (size_ != frame_.size())
        {
            return false;
        }

        bool valid = crc::verify_crc16_checksum(frame_.data(), vision_command_size) != 0;
        for (std::size_t offset = 2; valid && offset < frame_.size() - 2; offset += 4)
        {
            valid = finite_float_le(frame_.data() + offset);
        }
        if (valid)
        {
            command = {frame_[1], read_float_le(frame_.data() + 2),
                read_float_le(frame_.data() + 6), read_float_le(frame_.data() + 10),
                read_float_le(frame_.data() + 14), read_float_le(frame_.data() + 18),
                read_float_le(frame_.data() + 22)};
            reset();
            return true;
        }

        // A damaged frame may already contain the beginning of the next valid frame.
        std::size_t next = 1;
        while (next < size_ && frame_[next] != 0xA5)
        {
            ++next;
        }
        size_ -= next;
        std::memmove(frame_.data(), frame_.data() + next, size_);
        return false;
    }

    void reset()
    {
        size_ = 0;
    }

private:
    std::array<uint8_t, vision_command_size> frame_{};
    std::size_t size_{};
};

inline std::array<uint8_t, vision_telemetry_size> encode_vision(const vision_telemetry& value)
{
    std::array<uint8_t, vision_telemetry_size> bytes{};
    bytes[0] = 0x5A;
    bytes[1] = static_cast<uint8_t>(value.detect_color | (value.reset_tracker << 1U));
    bytes[2] = value.mode;
    // Byte 3 is the old reserved bit-field allocation byte, not part of the quaternion.
    for (unsigned i = 0; i < 4; ++i)
    {
        write_float_le(bytes.data() + 4 + i * 4, value.q[i]);
    }
    write_float_le(bytes.data() + 20, value.yaw);
    write_float_le(bytes.data() + 24, value.gyro_yaw);
    write_float_le(bytes.data() + 28, value.pitch);
    write_float_le(bytes.data() + 32, value.gyro_pitch);
    write_float_le(bytes.data() + 36, value.bullet_speed);
    bytes[40] = static_cast<uint8_t>(value.bullet_count);
    bytes[41] = static_cast<uint8_t>(value.bullet_count >> 8U);
    crc::append_crc16_checksum(bytes.data(), vision_telemetry_size);
    return bytes;
}

inline bool decode_lower_status(const uint8_t* data, std::size_t size, lower_status& status)
{
    if (data == nullptr || size != 8 || !finite_float_le(data + 3))
    {
        return false;
    }
    const uint32_t bits = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8U) |
                          (static_cast<uint32_t>(data[2]) << 16U);
    status = {(bits & 1U) != 0, (bits & 2U) != 0, static_cast<uint8_t>((bits >> 2U) & 0x0FU),
        static_cast<uint16_t>((bits >> 6U) & 0x1FFU), static_cast<uint16_t>((bits >> 15U) & 0x1FFU),
        read_float_le(data + 3)};
    return true;
}

inline std::array<uint8_t, 8> encode_lower_command(const lower_command& value)
{
    std::array<uint8_t, 8> bytes{};
    bytes[0] = static_cast<uint8_t>(value.ifmove | (value.ifjump << 1U) | (value.ifspin << 2U) |
                                    (value.ifturn << 3U) | (value.ifstair << 4U) |
                                    (value.iffly << 5U) | (value.ifgimbalonly << 6U));
    const auto current = static_cast<uint16_t>(value.yaw_cur);
    bytes[1] = static_cast<uint8_t>(current);
    bytes[2] = static_cast<uint8_t>(current >> 8U);
    bytes[3] = static_cast<uint8_t>(value.vx);
    bytes[4] = static_cast<uint8_t>(value.vy);
    bytes[5] = static_cast<uint8_t>(value.len_level);
    bytes[6] = static_cast<uint8_t>(value.tri_spd);
    return bytes;
}

inline std::array<uint8_t, 8> encode_lower_ui(const lower_ui& value)
{
    std::array<uint8_t, 8> bytes{};
    bytes[0] =
        static_cast<uint8_t>(value.reset | (value.fire << 1U) | (value.bulletfreq << 2U) |
                             (value.aim_rune << 3U) | ((value.aim_target_now & 0x0FU) << 4U));
    bytes[1] = value.aim_target_x;
    bytes[2] = value.aim_target_y;
    return bytes;
}

} // namespace big_head
