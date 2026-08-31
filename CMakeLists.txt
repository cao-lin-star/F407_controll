cmake_minimum_required(VERSION 3.16)
project(chassis_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(chassis_core_tests
    test/test_core.c
    src/crc16.c
    src/protocol.c
    src/pid.c
    src/kinematics.c
)
target_include_directories(chassis_core_tests PRIVATE include)
if(NOT MSVC)
    target_compile_options(chassis_core_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(chassis_core_tests PRIVATE m)
endif()

enable_testing()
add_test(NAME chassis_core_tests COMMAND chassis_core_tests)

