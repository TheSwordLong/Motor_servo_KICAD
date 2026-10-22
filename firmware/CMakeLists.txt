cmake_minimum_required(VERSION 3.22)

project(GD32F303VET6_MotorControl LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(TARGET_NAME ${PROJECT_NAME})

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

file(GLOB PERIPHERAL_SOURCES CONFIGURE_DEPENDS
    Drivers/GD32F30x_standard_peripheral/Source/*.c
)

add_executable(${TARGET_NAME}
    Core/Src/main.c
    Core/Src/gd32f30x_it.c
    Core/Src/systick.c

    BSP/button.c
    BSP/can_bus.c
    BSP/capture.c
    BSP/led.c
    BSP/motor_io.c
    BSP/pwm.c
    BSP/robot.c
    BSP/robot_protocol.c
    BSP/robot_rtu.c
    BSP/robot_store.c
    BSP/robot_port.c
    BSP/rs485.c
    BSP/usart0.c
    BSP/usart2.c

    Drivers/CMSIS/system_gd32f30x.c

    startup/startup_gd32f30x_hd.S

    Middlewares/Newlib/syscalls.c

    ${PERIPHERAL_SOURCES}
)

set_target_properties(${TARGET_NAME} PROPERTIES
    SUFFIX ".elf"
    LINK_DEPENDS "${CMAKE_SOURCE_DIR}/linker/gd32f303xE_flash.ld"
)

target_include_directories(${TARGET_NAME} PRIVATE
    Core/Inc
    BSP
    Drivers/CMSIS
    Drivers/GD32F30x_standard_peripheral/Include
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    GD32F30X_HD
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${MCU_FLAGS}
    -Og
    -g3
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
)

target_link_options(${TARGET_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/linker/gd32f303xE_flash.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
    --specs=nano.specs
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
        -O ihex
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex

    COMMAND ${CMAKE_OBJCOPY}
        -O binary
        $<TARGET_FILE:${TARGET_NAME}>
        ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin

    COMMAND ${CMAKE_SIZE}
        $<TARGET_FILE:${TARGET_NAME}>
)
