#include "iNVNt/core/logger.hpp"
#include "iNVNt/core/param_server.hpp"

using namespace nvn;
int main(int argc, char **argv)
{
    auto log = Logger();

    log << "Testing param/param.hpp \n";

    auto param = ParameterServer();

    param.setParamValue("my_int", 6);
    log << "Added 'my_int' to parameters\n";

    param.setParamValue("my_string", std::string("hello"));
    log << "Added 'my_string' to parameters\n";

    int my_int;
    if (param.getParamValue("my_int", my_int))
        log << "Retrieved 'my_int': " << my_int << "\n";
    else
        log << "Failed to retrieve 'my_int'\n";

    std::string my_string;
    if (param.getParamValue("my_string", my_string))
        log << "Retrieved 'my_string': " << my_string << "\n";
    else
        log << "Failed to retrieve 'my_string'\n";

    param.setParamValue("my_int_ref", new int{10});
    log << "Added 'my_int_ref' to parameters\n";

    int *my_int_ref;
    if (param.getParamValue("my_int_ref", my_int_ref))
        log << "Retrieved 'my_int_ref': " << *my_int_ref << "\n";
    else
        log << "Failed to retrieve 'my_int_ref'\n";

    *my_int_ref = 7;
    log << "Changed 'my_int_ref' to 7\n";

    if (param.getParamValue("my_int_ref", my_int_ref))
        log << "Retrieved 'my_int_ref': " << *my_int_ref << "\n";
    else
        log << "Failed to retrieve 'my_int_ref'\n";

    return 0;
}