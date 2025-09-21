#pragma once

#include <chrono>

#include "iNVNt/core/clock_timer.hpp"

namespace nvn
{
    struct ReadSystemTime
    {
        inline seconds_t operator()() const noexcept
        {
            static auto time_init = std::chrono::high_resolution_clock::now();
            return std::chrono::duration_cast<std::chrono::duration<seconds_t>>(
                       std::chrono::high_resolution_clock::now() - time_init)
                .count();
        }
    };

    template <TimeReader ReadTime = ReadSystemTime>
    struct Clock
    {
        ReadTime read_time;
        Timer<ReadTime> timer;

        template <typename... Args>
        Clock(Args &&...args)
            : read_time(std::forward<Args>(args)...),
              timer(Timer<ReadTime>(std::forward<ReadTime>(read_time))) {}

        inline seconds_t operator()() const noexcept
        {
            return read_time();
        }

        inline seconds_t now() const noexcept
        {
            return read_time();
        }

        inline seconds_t time() const noexcept
        {
            return timer.seconds();
        }

        inline seconds_t lap() noexcept
        {
            return timer.lap();
        }
    };

    using SystemClock = Clock<ReadSystemTime>;
}