add_executable(bench
  bench.cpp
  stepper.cpp
)

target_link_libraries(bench PUBLIC pico_stdlib
                            PUBLIC sdfat
                            PUBLIC SDIO_RP2350
                            PUBLIC hardware_clocks 
                            PUBLIC hardware_pwm
)

target_include_directories(bench PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR})

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(bench)
