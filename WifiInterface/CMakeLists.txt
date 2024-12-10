# cmake_minimum_required(VERSION 3.13)

# set(PICO_BOARD pico_w)

# include(../pico_sdk_import.cmake)


set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(lwipopts_provider INTERFACE)
target_include_directories(lwipopts_provider INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)
add_executable(WifiInterface
  ak8419.cpp
  tiff.cpp
  main.cpp
  camera.cpp

  include/dhcpserver.c
  include/dnsserver.c
)
target_include_directories(WifiInterface PRIVATE ${CMAKE_CURRENT_LIST_DIR}/include)
pico_generate_pio_header(WifiInterface ${CMAKE_CURRENT_LIST_DIR}/ak8419.pio)
pico_generate_pio_header(WifiInterface ${CMAKE_CURRENT_LIST_DIR}/step.pio)

target_link_libraries(WifiInterface
  pico_stdlib
  pico_cyw43_arch_lwip_poll

  # pico_cyw43_arch_lwip_threadsafe_background
  pico_ws_server
  pico_multicore
  sdfat
  sdio
  hardware_pio hardware_dma hardware_pwm
)

target_compile_definitions(WifiInterface PRIVATE
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  )

pico_add_extra_outputs(WifiInterface)
