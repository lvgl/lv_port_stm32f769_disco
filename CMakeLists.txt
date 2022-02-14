cmake_minimum_required(VERSION 3.10)

include("ide/cmake/arm-none-eabi.cmake")

# project name
project(lv_stm32f769 C ASM)

# Generate compile_commands.json file 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# select linker script
set(LINKER_SCRIPT "LinkerScript.ld")

# define CPU OPTIONS
set(CPU_OPTIONS -mthumb -march=armv7e-m)

# optimizations (-O0 -O1 -O2 -O3 -Os -Ofast -Og -flto)
set(CMAKE_C_FLAGS_DEBUG "-Og -g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-Og -g")
set(CMAKE_C_FLAGS_RELEASE "-Os")
set(CMAKE_CXX_FLAGS_RELEASE "-Os")
set(CMAKE_ASM_SOURCE_FILE_EXTENSIONS s)

add_compile_options(
    ${CPU_OPTIONS}
    $<$<COMPILE_LANGUAGE:C>:-std=gnu99>
    $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>
    $<$<COMPILE_LANGUAGE:CXX>:-fms-extensions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
    -fstrict-volatile-bitfields
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wshadow
)

include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/CMSIS/device
    ${CMAKE_SOURCE_DIR}/CMSIS/core
    ${CMAKE_SOURCE_DIR}/lvgl
    ${CMAKE_SOURCE_DIR}/Utilities/STM32F769I-Discovery
)

link_directories(
    ${CMAKE_SOURCE_DIR}
)

FILE(GLOB_RECURSE HAL_Sources HAL_Driver/Src/*.c
    Utilities/Components/ft6x06/ft6x06.c
    Utilities/Components/otm8009a/otm8009a.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_ts.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_lcd.c
    Utilities/STM32F769I-Discovery/stm32f769i_discovery_sdram.c
)

FILE(GLOB_RECURSE LVGL_Sources CONFIGURE_DEPENDS hal_stm_lvgl/*.c)

add_subdirectory(lvgl)

add_definitions(
    -DSTM32
    -DSTM32F7
    -DSTM32F769xx
    -DSTM32F769I_DISCO
    -DDEBUG
    -DUSE_HAL_DRIVER
)

# list of modules to build final firmware (without extension .c or .cpp)
add_executable(${PROJECT_NAME}
    src/main.c
    src/stm32f7xx_it.c
    src/syscalls.c
    src/system_stm32f7xx.c
    startup/startup_stm32f769xx.s
    ${HAL_Sources}
    ${LVGL_Sources}
)

target_link_libraries(${PROJECT_NAME}
    ${CPU_OPTIONS}
    -T${LINKER_SCRIPT}
    -nodefaultlibs
    -Wl,--gc-sections
    m
    lvgl
    lvgl_demos
    # -nostdlib
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX .elf
    LINK_DEPENDS "${LINKER_SCRIPTS}"
)

add_custom_target(${PROJECT_NAME}.bin ALL DEPENDS ${PROJECT_NAME})
add_custom_command(TARGET ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_C_OBJCOPY} ARGS -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin)

set_property(TARGET ${PROJECT_NAME} PROPERTY LINK_DEPENDS ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT})