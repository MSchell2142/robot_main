# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)
project(main_file C CXX ASM)
pico_sdk_init()

# Arducam Mega library under lib/Arducam_Mega/src/ and src/Arducam/,
# with Arduino-only ArducamSpi.cpp filtered out.
set(ARDUCAM_ROOT "${CMAKE_CURRENT_LIST_DIR}/lib/Arducam_Mega/src")
file(GLOB ARDUCAM_SOURCES
    CONFIGURE_DEPENDS
    "${ARDUCAM_ROOT}/*.c"
    "${ARDUCAM_ROOT}/*.cpp"
    "${ARDUCAM_ROOT}/Arducam/*.c"
    "${ARDUCAM_ROOT}/Arducam/*.cpp"
)
list(FILTER ARDUCAM_SOURCES EXCLUDE REGEX ".*/ArducamSpi\\.cpp$")
message(STATUS "Arducam sources after filter: ${ARDUCAM_SOURCES}")

add_executable(main_file
    main.cpp
    camera.cpp
    encoders.cpp
    vision.c
    cam_spi_master.c
    field.cpp
    sonar.cpp
    ${ARDUCAM_SOURCES}
)

pico_set_program_name(main_file "ball_tracker")
pico_set_program_version(main_file "0.1")

pico_enable_stdio_uart(main_file 0)
pico_enable_stdio_usb(main_file 1)

target_link_libraries(main_file
    pico_stdlib
    hardware_spi
    hardware_pwm
    hardware_i2c
    hardware_gpio
    hardware_irq
    hardware_sync
)

target_include_directories(main_file PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${ARDUCAM_ROOT}
    ${ARDUCAM_ROOT}/Arducam
)

pico_add_extra_outputs(main_file)