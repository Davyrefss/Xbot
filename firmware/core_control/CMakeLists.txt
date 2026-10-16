cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME core_control)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 链接选项
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -Wl,--no-warn-rwx-segments)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add user defined sources 设置文件源
file(GLOB SOURCE_APP CONFIGURE_DEPENDS "source/app/*.c")
file(GLOB SOURCE_SERVICE CONFIGURE_DEPENDS "source/service/*.c")
file(GLOB SOURCE_OSAL CONFIGURE_DEPENDS "source/osal/*.c")
file(GLOB SOURCE_BSP CONFIGURE_DEPENDS "source/bsp/*.c")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${SOURCE_APP}
    ${SOURCE_SERVICE}
    ${SOURCE_OSAL}
    ${SOURCE_BSP}
    
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    source/app
    source/service
    source/osal
    source/bsp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Locate arm-none-eabi-objcopy next to the compiler (compiler path is absolute,
# so this does not depend on the build shell's PATH).
get_filename_component(TOOLCHAIN_BIN_DIR "${CMAKE_C_COMPILER}" DIRECTORY)
find_program(OBJCOPY_TOOL arm-none-eabi-objcopy HINTS "${TOOLCHAIN_BIN_DIR}" NO_DEFAULT_PATH)

# Generate binary (.bin) and Intel Hex (.hex) files for flashing
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${OBJCOPY_TOOL} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${OBJCOPY_TOOL} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
)
