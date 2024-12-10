# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
file(READ static.html.gz PICO_WS_SERVER_STATIC_HTML_HEX HEX)
add_compile_options( 
    -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(PicoPan C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
add_subdirectory(libs)

# Add executable. Default name is the project name, version 0.1

add_executable(PicoPan 
ak8419.cpp
tiff.cpp
main.cpp
camera.cpp

include/dhcpserver.c
include/dnsserver.c )

pico_set_program_name(PicoPan "PicoPan")
pico_set_program_version(PicoPan "0.1")

# Generate PIO header
target_include_directories(PicoPan PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
pico_generate_pio_header(PicoPan ${CMAKE_CURRENT_LIST_DIR}/ak8419.pio)
pico_generate_pio_header(PicoPan ${CMAKE_CURRENT_LIST_DIR}/step.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PicoPan 0)
pico_enable_stdio_usb(PicoPan 1)

# Add the standard library to the build
target_link_libraries(PicoPan
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(PicoPan PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(PicoPan 
        pico_ws_server
        pico_multicore
        sdfat
        sdio
        hardware_pio hardware_dma hardware_pwm hardware_clocks
        pico_cyw43_driver
        pico_cyw43_arch_lwip_poll
        
        )
        target_compile_definitions(PicoPan PRIVATE
        WIFI_SSID=\"${WIFI_SSID}\"
        WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
        )
pico_add_extra_outputs(PicoPan)

