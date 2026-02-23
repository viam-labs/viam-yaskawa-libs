cmake_minimum_required(VERSION 3.14)
project(comms VERSION 1.0.0 LANGUAGES C)

# Platform option — ON by default, links POSIX .c files
# Set to OFF when consumer provides their own platform implementation
option(COMMS_PLATFORM_POSIX "Include POSIX platform/lock implementations" ON)

# Resolve the real directory for logging (follows symlinks)
get_filename_component(LOGGING_REAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/logging" REALPATH)

# Core sources
set(COMMS_SOURCES
    robot_protocol.c
    version.c
)

# Logging sources
set(LOGGING_SOURCES
    ${LOGGING_REAL_DIR}/src/ring_buffer.c
    ${LOGGING_REAL_DIR}/src/logging.c
    ${LOGGING_REAL_DIR}/src/udp_broadcast.c
)

# POSIX platform sources (default)
set(POSIX_SOURCES
    lock_api_posix.c
    platform_api_posix.c
)

# Build library
set(ALL_SOURCES ${COMMS_SOURCES} ${LOGGING_SOURCES})
if(COMMS_PLATFORM_POSIX)
    list(APPEND ALL_SOURCES ${POSIX_SOURCES})
endif()

add_library(comms STATIC ${ALL_SOURCES})

target_include_directories(comms PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${LOGGING_REAL_DIR}/include>)

target_compile_options(comms PRIVATE -Wall -Wextra -Werror -std=gnu99)

if(COMMS_PLATFORM_POSIX)
    target_link_libraries(comms PRIVATE pthread m)
endif()

# Tests
option(COMMS_BUILD_TESTS "Build tests" ON)
if(COMMS_BUILD_TESTS)
    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
        unity
        GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG v2.6.1
        GIT_SHALLOW TRUE
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(unity)

    target_include_directories(unity PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tests/protocol>)
    target_compile_definitions(unity PUBLIC UNITY_INCLUDE_CONFIG_H)
    target_compile_options(unity PRIVATE -std=gnu99)

    # Protocol tests
    add_executable(test_robot_protocol
        tests/protocol/test_robot_protocol.c
        tests/protocol/mock_robot.c)
    target_include_directories(test_robot_protocol PRIVATE tests/protocol)
    target_link_libraries(test_robot_protocol PRIVATE comms unity pthread m)
    add_test(NAME robot_protocol_tests COMMAND test_robot_protocol)

    # Logging tests
    add_executable(test_ring_buffer tests/logging/test_ring_buffer.c)
    target_link_libraries(test_ring_buffer PRIVATE comms unity pthread m)
    add_test(NAME ring_buffer_tests COMMAND test_ring_buffer)

    add_executable(test_logging tests/logging/test_logging.c)
    target_link_libraries(test_logging PRIVATE comms unity pthread m)
    add_test(NAME logging_tests COMMAND test_logging)

    add_executable(test_simple tests/logging/test_simple.c)
    target_link_libraries(test_simple PRIVATE comms unity pthread m)
    add_test(NAME simple_tests COMMAND test_simple)
endif()

# Examples
option(COMMS_BUILD_EXAMPLES "Build examples" ON)
if(COMMS_BUILD_EXAMPLES)
    add_executable(robot_server_example
        tests/protocol/robot_server_example.c
        tests/protocol/mock_robot.c)
    target_include_directories(robot_server_example PRIVATE tests/protocol)
    target_link_libraries(robot_server_example PRIVATE comms pthread m)

    add_executable(test_udp_server tests/logging/test_udp_server.c)
    target_link_libraries(test_udp_server PRIVATE comms pthread m)

    add_executable(test_udp_client tests/logging/test_udp_client.c)
    target_link_libraries(test_udp_client PRIVATE comms pthread m)
endif()
