#pragma once

#include <sstream>
#include <iostream>
#include <fstream>

#include "iNVNt/core/func_concepts.hpp"

namespace nvn
{
    struct PrintConsole
    {
        inline void operator()(const std::string &s) const
        {
            std::cout.write(s.data(), static_cast<std::streamsize>(s.size()));
        }
    };

    struct PrintToFile
    {
        std::ofstream stream;

        explicit PrintToFile(const std::string &file_path,
                             std::ios_base::openmode mode = std::ios_base::app | std::ios_base::out)
            : stream(file_path, mode)
        {
            if (!stream.is_open())
            {
                throw std::runtime_error("PrintToFile: failed to open '" + file_path + "'");
            }
        }

        inline void operator()(const std::string &s) noexcept
        {
            stream.write(s.data(), static_cast<std::streamsize>(s.size()));
        }
    };

    template <StringWriter WriteStr = PrintConsole>
    struct Logger
    {
        WriteStr log_write;

        template <typename... Args>
        explicit Logger(Args &&...args)
            : log_write(std::forward<Args>(args)...) {}

        inline Logger &operator<<(const std::string &s)
        {
            log_write(s);
            return *this;
        }

        inline Logger &operator<<(const char *cstr)
        {
            log_write(cstr ? std::string{cstr} : std::string{});
            return *this;
        }

        template <typename _T>
        inline Logger &operator<<(const _T &val)
        {
            std::ostringstream oss;
            oss << val;
            log_write(oss.str());
            return *this;
        }

        inline void print(const std::string &s) { log_write(s); }

        inline void println(const std::string &s)
        {
            log_write(s);
            log_write("\n");
        }

        inline void bell()
        {
            log_write("\a");
        }
    };

    using ConsoleLog = Logger<PrintConsole>;
}
