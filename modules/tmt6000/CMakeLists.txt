
target_include_directories(tmt6000 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(tmt6000 STATIC
    tmt6000.c
)

target_link_libraries(tmt6000
    pico_stdlib
    hardware_adc
)
