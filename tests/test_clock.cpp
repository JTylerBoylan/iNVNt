#include "iNVNt/core/logger.hpp"
#include "iNVNt/core/clock.hpp"

#include <thread>

using namespace nvn;
int main(int argc, char **argv)
{
    auto log = Logger();
    // auto log = FileLog("/tmp/log.txt", std::ios_base::trunc | std::ios_base::out);
    log << "Testing clock/clock.hpp\n";

    auto clock = Clock();
    log << "Current time: " << clock.now() << " seconds\n";

    log << "Sleeping for 0.5 seconds\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    log << "Time elapsed: " << clock.time() << " seconds\n";

    return 0;
}