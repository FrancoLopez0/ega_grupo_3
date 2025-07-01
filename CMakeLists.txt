# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(ega_grupo_3 C CXX ASM)

# Agrega el subdirectorio con tus módulos
add_subdirectory(modules/tmt6000)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ega_grupo_3 ega_grupo_3.c )

pico_set_program_name(ega_grupo_3 "ega_grupo_3")
pico_set_program_version(ega_grupo_3 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ega_grupo_3 0)
pico_enable_stdio_usb(ega_grupo_3 1)

# Add the standard library to the build
target_link_libraries(ega_grupo_3
        pico_stdlib
        tmt6000
        )

# Add the standard include files to the build
target_include_directories(ega_grupo_3 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(ega_grupo_3 
        
        )

pico_add_extra_outputs(ega_grupo_3)

