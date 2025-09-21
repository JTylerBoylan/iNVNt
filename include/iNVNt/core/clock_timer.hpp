#pragma once

#include "iNVNt/core/math_concepts.hpp"

namespace nvn
{
    template <TimeReader ReadT>
    struct Timer
    {
        ReadT read_time;
        seconds_t t_start;
        seconds_t t_lap;

        explicit Timer(ReadT &&time)
            : read_time(std::forward<ReadT>(time))
        {
            t_lap = t_start = read_time();
        }

        inline seconds_t seconds() const noexcept
        {
            return read_time() - t_start;
        }

        inline seconds_t lap() noexcept
        {
            seconds_t now = read_time();
            seconds_t time_lap = now - t_lap;
            t_lap = now;
            return time_lap;
        }

        inline void reset() noexcept
        {
            t_lap = t_start = read_time();
        }
    };
}