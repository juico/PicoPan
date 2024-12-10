add_executable(ak8419
ak8419.pio
ak8419.cpp
step.pio
)
pico_generate_pio_header(ak8419 ${CMAKE_CURRENT_LIST_DIR}/ak8419.pio)
pico_generate_pio_header(ak8419 ${CMAKE_CURRENT_LIST_DIR}/step.pio)

pico_enable_stdio_uart(ak8419 0)
pico_enable_stdio_usb(ak8419 1)

target_link_libraries(ak8419 PUBLIC pico_stdlib
                            PUBLIC sdfat
                            PUBLIC sdio
                            hardware_pio hardware_dma hardware_pwm
)

target_include_directories(ak8419 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})
pico_set_program_name(ak8419 "ak8419")
pico_set_program_version(ak8419 "0.1")
# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(ak8419)
