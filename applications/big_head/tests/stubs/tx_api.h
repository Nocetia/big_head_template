#pragma once

using UINT = unsigned int;

inline constexpr UINT TX_INT_DISABLE = 1U;

struct TX_SEMAPHORE
{
};

inline UINT tx_interrupt_control(UINT posture) noexcept
{
    return posture;
}
