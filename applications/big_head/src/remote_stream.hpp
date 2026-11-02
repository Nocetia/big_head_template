#pragma once

#include "remoter.hpp"
#include <cstring>

namespace big_head
{
// UART callback boundaries are not frame boundaries. The caller discards this
// partial frame on a receive gap, overflow or UART restart.
class remote_stream
{
public:
    explicit remote_stream(bool vt03) : vt03_(vt03)
    {
    }

    void reset() noexcept
    {
        size_ = 0;
    }

    bool push(uint8_t byte, remotes::reading& reading)
    {
        bytes_[size_++] = byte;
        const auto frame_size = vt03_ ? remotes::vt03::frame_size : remotes::dr16::frame_size;
        if (size_ < frame_size)
        {
            return false;
        }
        const auto result = vt03_ ? remotes::vt03::decode(bytes_, size_, reading)
                                  : remotes::dr16::decode(bytes_, size_, reading);
        if (result == pnxtypes::status::ok)
        {
            size_ = 0;
            return true;
        }
        --size_;
        std::memmove(bytes_, bytes_ + 1, size_);
        return false;
    }

private:
    bool vt03_;
    uint8_t bytes_[remotes::vt03::frame_size]{};
    uint16_t size_ = 0;
};
} // namespace big_head
