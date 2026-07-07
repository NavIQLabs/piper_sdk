cmake_minimum_required(VERSION 3.10)
project(cpiper C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -O2)

# Library (static)
add_library(cpiper STATIC
    src/protocol.c
    src/can.c
    src/param_manager.c
    src/kinematics.c
    src/fps.c
    src/interface.c
)

# Shared library for Python ctypes binding
add_library(cpiper_shared SHARED
    src/protocol.c
    src/can.c
    src/param_manager.c
    src/kinematics.c
    src/fps.c
    src/interface.c
)
set_target_properties(cpiper_shared PROPERTIES OUTPUT_NAME cpiper)

target_include_directories(cpiper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(cpiper_shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(cpiper PRIVATE pthread m)
target_link_libraries(cpiper_shared PRIVATE pthread m)

# Example: send_zeros
add_executable(send_zeros examples/send_zeros.c)
target_link_libraries(send_zeros PRIVATE cpiper pthread m)

# Tests
enable_testing()

add_executable(test_protocol tests/test_protocol.c)
target_link_libraries(test_protocol PRIVATE cpiper m)
add_test(NAME protocol COMMAND test_protocol)

add_executable(test_can tests/test_can.c)
target_link_libraries(test_can PRIVATE cpiper pthread m)
add_test(NAME can COMMAND test_can)

add_executable(test_interface tests/test_interface.c)
target_link_libraries(test_interface PRIVATE cpiper pthread m)
add_test(NAME interface COMMAND test_interface)

add_executable(test_compare_helper tests/test_compare_helper.c)
target_link_libraries(test_compare_helper PRIVATE cpiper pthread m)
