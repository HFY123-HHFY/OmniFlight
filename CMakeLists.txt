cmake_minimum_required(VERSION 3.22)

project(OmniLayer C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# 方便 VS Code/clangd 索引。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 统一输出目录，便于管理产物。
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/artifacts)

# ============================================================================
# 目标平台：STM32F407VET6（毕设主线仅此一个目标）
#
# 架构上仍通过 Enroll 注册层 + X-Macro 实现算法与芯片解耦；
# F103 / MSPM0G3507 的历史移植已归档于 git tag：archive/multi-mcu-f103-g3507
# ============================================================================

# I2C 实现切换：SOFT (bit-bang) / HARD (片上 I2C1)，默认从 BusRate.h 读取
set(I2C_MODE "AUTO" CACHE STRING "I2C implementation: AUTO (read BusRate.h) / SOFT / HARD")
set_property(CACHE I2C_MODE PROPERTY STRINGS AUTO SOFT HARD)

# AUTO 模式下，从 BusRate.h 读取 I2C_MODE。
if(I2C_MODE STREQUAL "AUTO")
    file(READ "${CMAKE_SOURCE_DIR}/SYSTEM/BusRate.h" BUSRATE_CONTENT)
    string(REGEX MATCH "(^|[\r\n])[ \t]*#define[ \t]+I2C_MODE[ \t]+I2C_MODE_HARD([ \t]|[\r\n]|$)" I2C_IS_HARD "${BUSRATE_CONTENT}")
    if(I2C_IS_HARD)
        set(RESOLVED_I2C_MODE "HARD")
    else()
        set(RESOLVED_I2C_MODE "SOFT")
    endif()
else()
    set(RESOLVED_I2C_MODE "${I2C_MODE}")
endif()
message(STATUS "Resolved I2C mode: ${RESOLVED_I2C_MODE}")

# ============================== 维护说明 ==============================
#
# 1) APP、API、SYS、ENROLL、BSP 层为统一源文件列表（见下方变量）。
# 2) MCU_SOURCES 为 F407 平台专属底层实现。
# 3) 改完后验证：
#    - 重新 Configure/Build；若报“undefined reference”通常是漏加 .c 文件。
#    - 若报“header not found”通常是漏加头文件目录到 MCU_INCLUDE_DIRS。
# =====================================================================

# 应用层（app）：业务入口与任务调度。
set(APP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/A_Entry/main.c
    ${CMAKE_SOURCE_DIR}/app/Control/Control.c
    ${CMAKE_SOURCE_DIR}/app/My_Usart/My_Usart.c
    ${CMAKE_SOURCE_DIR}/app/Control_Task/Control_Task.c
    ${CMAKE_SOURCE_DIR}/app/PID/PID.c
    ${CMAKE_SOURCE_DIR}/app/Filter/Filter.c
    ${CMAKE_SOURCE_DIR}/app/IMU/IMU.c
    ${CMAKE_SOURCE_DIR}/app/Altitude/Altitude.c
)

# 系统层（SYSTEM）：中断管理与板级系统封装。
set(SYS_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/SYSTEM/sys.c
)

# API 层：对上提供统一外设接口，对下分发到 MCU Core 实现。
# 注意：i2c_soft.c 仅在 SOFT 模式下追加（见下方 I2C 模式选择块）。
set(API_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/API/src/adc.c
    ${CMAKE_SOURCE_DIR}/API/src/exti.c
    ${CMAKE_SOURCE_DIR}/API/src/gpio.c
    ${CMAKE_SOURCE_DIR}/API/src/tim.c
    ${CMAKE_SOURCE_DIR}/API/src/usart.c
    ${CMAKE_SOURCE_DIR}/API/src/pwm.c
    ${CMAKE_SOURCE_DIR}/API/src/Encoder.c
    ${CMAKE_SOURCE_DIR}/API/API_I2C/API_I2C.c
    ${CMAKE_SOURCE_DIR}/API/API_SPI/API_SPI.c
)

# 注册层（Enroll）：把 BSP 资源和具体 MCU 驱动绑定。
set(ENROLL_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/Enroll/Enroll.c
)

# 通用头文件目录。
set(COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/A_Entry
    ${CMAKE_SOURCE_DIR}/app
    ${CMAKE_SOURCE_DIR}/app/Control_Task
    ${CMAKE_SOURCE_DIR}/app/Control
    ${CMAKE_SOURCE_DIR}/app/My_Usart
    ${CMAKE_SOURCE_DIR}/app/PID
    ${CMAKE_SOURCE_DIR}/app/Filter
    ${CMAKE_SOURCE_DIR}/app/IMU
    ${CMAKE_SOURCE_DIR}/app/Altitude
    ${CMAKE_SOURCE_DIR}/API
    ${CMAKE_SOURCE_DIR}/API/inc
    ${CMAKE_SOURCE_DIR}/API/API_I2C
    ${CMAKE_SOURCE_DIR}/API/API_SPI
    ${CMAKE_SOURCE_DIR}/Enroll
    ${CMAKE_SOURCE_DIR}/SYSTEM
)

# 通用 BSP 头文件目录。
set(BSP_COMMON_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/BSP/LED
    ${CMAKE_SOURCE_DIR}/BSP/KEY
    ${CMAKE_SOURCE_DIR}/BSP/OLED
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL
    ${CMAKE_SOURCE_DIR}/BSP/TB6612
)

# 板级支持层（BSP）：对传感器/执行器做硬件抽象。
set(BSP_LAYER_SOURCES
    ${CMAKE_SOURCE_DIR}/BSP/LED/LED.c
    ${CMAKE_SOURCE_DIR}/BSP/KEY/KEY.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED.c
    ${CMAKE_SOURCE_DIR}/BSP/OLED/OLED_Data.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/MPU6050.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/MPU6050_Int.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL/inv_mpu.c
    ${CMAKE_SOURCE_DIR}/BSP/MPU6050/eMPL/inv_mpu_dmp_motion_driver.c
    ${CMAKE_SOURCE_DIR}/BSP/TB6612/TB6612.c
    ${CMAKE_SOURCE_DIR}/BSP/BMP280/BMP280.c
    ${CMAKE_SOURCE_DIR}/BSP/NRF24L01/NRF24L01.c
    ${CMAKE_SOURCE_DIR}/BSP/QMC5883P/QMC5883P.c
    ${CMAKE_SOURCE_DIR}/BSP/Dshot/Dshot.c
    ${CMAKE_SOURCE_DIR}/BSP/Motor/Motor.c
    ${CMAKE_SOURCE_DIR}/BSP/Buzzer/Buzzer.c
    ${CMAKE_SOURCE_DIR}/BSP/STP23L/STP23L.c
)

###############################################################
# F407 平台配置与源文件
###############################################################
set(TARGET_NAME OmniLayer_F407)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/Core/STM32F407/cmake/STM32F407VE_FLASH.ld)
set(OPENOCD_CFG ${CMAKE_SOURCE_DIR}/OpenOCD/F407_OpenOCD.cfg)
set(OPENOCD_ERASE_CMD "stm32f4x mass_erase 0")
set(MCU_CPU_FLAGS -mcpu=cortex-m4 -mthumb)
set(MCU_FPU_FLAGS -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
set(MCU_DEFINITIONS STM32F40_41xxx)

set(MCU_INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
    ${BSP_COMMON_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/BSP/BMP280
    ${CMAKE_SOURCE_DIR}/BSP/NRF24L01
    ${CMAKE_SOURCE_DIR}/BSP/QMC5883P
    ${CMAKE_SOURCE_DIR}/BSP/Dshot
    ${CMAKE_SOURCE_DIR}/BSP/Motor
    ${CMAKE_SOURCE_DIR}/BSP/Buzzer
    ${CMAKE_SOURCE_DIR}/BSP/STP23L
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/inc
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_spi
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start
)

# F407 平台专属 MCU 驱动（I2C 在下方切换块中条件追加）
set(MCU_SOURCES
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_delay.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_spi/f407_soft_spi.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_exti.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_adc.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_gpio.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_tim.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_usart.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_pwm.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_dma.c
    ${CMAKE_SOURCE_DIR}/Core/STM32F407/src/f407_Encoder.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start/system_stm32f4xx.c
    ${CMAKE_SOURCE_DIR}/Drivers/Drivers_STM32F4/Start/startup_stm32f407vetx_gcc.c
)

# ═══════════════════════════════════════════════════════════
# I2C 模式切换：SOFT (bit-bang) / HARD (片上 I2C1)
# ═══════════════════════════════════════════════════════════
if(RESOLVED_I2C_MODE STREQUAL "HARD")
    list(APPEND MCU_SOURCES
        ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_hw_i2c/f407_hw_i2c.c
    )
    list(APPEND MCU_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_hw_i2c
    )
else()
    list(APPEND API_LAYER_SOURCES
        ${CMAKE_SOURCE_DIR}/API/API_I2C/i2c_soft.c
    )
    list(APPEND MCU_SOURCES
        ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_i2c/f407_soft_i2c.c
    )
    list(APPEND MCU_INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/Core/STM32F407/f407_soft_i2c
    )
endif()

###############################################################
# FreeRTOS 内核（Middlewares/FreeRTOS-Kernel）— Cortex-M4F + heap_4
# 调度器已在 main.c 中启动（配置见 A_Entry/FreeRTOSConfig.h），飞控控制链由 6 个 RTOS 任务接管。
###############################################################
set(FREERTOS_KERNEL_SOURCES
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/tasks.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/queue.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/list.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/timers.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/event_groups.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/stream_buffer.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/port.c
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/portable/MemMang/heap_4.c
)
set(FREERTOS_KERNEL_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/include
    ${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS-Kernel/portable/GCC/ARM_CM4F
)

###############################################################
# 汇总所有层的源文件
###############################################################
set(TARGET_SOURCES
    ${APP_LAYER_SOURCES}
    ${SYS_LAYER_SOURCES}
    ${API_LAYER_SOURCES}
    ${ENROLL_LAYER_SOURCES}
    ${BSP_LAYER_SOURCES}
    ${MCU_SOURCES}
    ${FREERTOS_KERNEL_SOURCES}
)

add_executable(${TARGET_NAME} ${TARGET_SOURCES})

target_include_directories(${TARGET_NAME} PRIVATE
    ${MCU_INCLUDE_DIRS}
    ${FREERTOS_KERNEL_INCLUDE_DIRS}
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    ${MCU_DEFINITIONS}
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -Wall
    -Wextra
)

target_link_options(${TARGET_NAME} PRIVATE
    ${MCU_CPU_FLAGS}
    ${MCU_FPU_FLAGS}
    -T${LINKER_SCRIPT}
    --specs=nosys.specs
    --specs=nano.specs
    -u
    _printf_float
    -Wl,--gc-sections
    -Wl,-Map=${TARGET_NAME}.map
    -Wl,--print-memory-usage
)

target_link_libraries(${TARGET_NAME} PRIVATE
    m
    c
)

###############################################################
# 生成 hex/bin 及打印大小
###############################################################
add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generate .hex/.bin and print size"
)

###############################################################
# OpenOCD 烧录/擦除
###############################################################
add_custom_target(flash
    COMMAND openocd -f ${OPENOCD_CFG} -c "program $<TARGET_FILE:${TARGET_NAME}> verify reset exit"
    DEPENDS ${TARGET_NAME}
    COMMENT "Flash firmware via OpenOCD"
)

add_custom_target(erase
    COMMAND openocd -f ${OPENOCD_CFG} -c "init; reset halt; ${OPENOCD_ERASE_CMD}; reset run; exit"
    COMMENT "Mass erase firmware via OpenOCD"
)
