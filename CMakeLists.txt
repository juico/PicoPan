cmake_minimum_required(VERSION 3.13)
file(READ static.html.gz PICO_WS_SERVER_STATIC_HTML_HEX HEX)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(WifiScanCam)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_cxx)
set(PICO_BOARD pico_w)
string(APPEND CMAKE_EXE_LINKER_FLAGS "-Wl,--print-memory-usage")
if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()

add_subdirectory(SdFat)
add_subdirectory(sdio)
add_subdirectory(bench)
add_subdirectory(ws-server)
add_subdirectory(WifiInterface)
add_compile_options( 
    -Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )

set_property(TARGET WifiInterface APPEND_STRING PROPERTY LINK_FLAGS "-Wl,--print-memory-usage")

pico_enable_stdio_usb(bench 1)
pico_enable_stdio_uart(bench 0)
pico_enable_stdio_usb(WifiInterface 1)
pico_enable_stdio_uart(WifiInterface 0)


