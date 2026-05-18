cmake_minimum_required(VERSION 3.14)
project(comms VERSION 1.0.0 LANGUAGES C)

# Platform option — ON by default, links POSIX .c files
# Set to OFF when consumer provides their own platform implementation
option(COMMS_PLATFORM_POSIX "Include POSIX platform/lock implementations" ON)

# Version mismatch policy: 0=ignore, 1=warn, 2=reject
set(COMMS_VERSION_MISMATCH "1" CACHE STRING "Version mismatch policy: 0=ignore, 1=warn, 2=reject")

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
target_compile_definitions(comms PRIVATE COMMS_VERSION_MISMATCH=${COMMS_VERSION_MISMATCH})

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

    # Protocol tests — build all 3 version mismatch variants (ignore=0, warn=1, reject=2)
    set(MISMATCH_NAMES ignore warn reject)
    set(MISMATCH_VALUES 0 1 2)
    foreach(MODE_NAME MODE_VALUE IN ZIP_LISTS MISMATCH_NAMES MISMATCH_VALUES)
        # Library variant with this mismatch policy
        add_library(comms_${MODE_NAME} STATIC ${ALL_SOURCES})
        target_include_directories(comms_${MODE_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
            $<BUILD_INTERFACE:${LOGGING_REAL_DIR}/include>)
        target_compile_options(comms_${MODE_NAME} PRIVATE -Wall -Wextra -Werror -std=gnu99)
        target_compile_definitions(comms_${MODE_NAME} PRIVATE COMMS_VERSION_MISMATCH=${MODE_VALUE})
        if(COMMS_PLATFORM_POSIX)
            target_link_libraries(comms_${MODE_NAME} PRIVATE pthread m)
        endif()

        # Test executable linked to the matching library variant
        add_executable(test_robot_protocol_${MODE_NAME}
            tests/protocol/test_robot_protocol.c
            tests/protocol/mock_robot.c)
        target_include_directories(test_robot_protocol_${MODE_NAME} PRIVATE tests/protocol)
        target_link_libraries(test_robot_protocol_${MODE_NAME} PRIVATE comms_${MODE_NAME} unity pthread m)
        target_compile_definitions(test_robot_protocol_${MODE_NAME} PRIVATE COMMS_VERSION_MISMATCH=${MODE_VALUE})
        add_test(NAME robot_protocol_tests_${MODE_NAME} COMMAND test_robot_protocol_${MODE_NAME})
    endforeach()

    # Fault injection tests
    add_executable(test_fault_injection
        tests/protocol/test_fault_injection.c
        tests/protocol/mock_robot.c
        tests/protocol/fault_inject.c)
    target_include_directories(test_fault_injection PRIVATE tests/protocol)
    target_link_libraries(test_fault_injection PRIVATE comms_warn unity pthread m)
    target_compile_definitions(test_fault_injection PRIVATE COMMS_VERSION_MISMATCH=1)
    add_test(NAME fault_injection_tests COMMAND test_fault_injection)

    # Recv-loop underflow regression test (task #6).
    # Uses -Wl,--wrap=platform_recv to drive the buggy payload-read loop in
    # tcp_thread_func with a stubbed recv returning -1 and asserts on the
    # uint32_t underflow of payload_received_bytes. CTest TIMEOUT bounds the
    # run as a belt-and-braces safety net on top of the in-stub iteration cap.
    add_executable(test_recv_underflow
        tests/protocol/test_recv_underflow.c)
    target_include_directories(test_recv_underflow PRIVATE tests/protocol)
    target_link_libraries(test_recv_underflow PRIVATE comms_warn unity pthread m)
    target_compile_definitions(test_recv_underflow PRIVATE COMMS_VERSION_MISMATCH=1)
    target_link_options(test_recv_underflow PRIVATE "LINKER:--wrap=platform_recv")
    add_test(NAME recv_underflow_regression COMMAND test_recv_underflow)
    set_tests_properties(recv_underflow_regression PROPERTIES TIMEOUT 10)

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
