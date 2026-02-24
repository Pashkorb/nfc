cmake_minimum_required(VERSION 3.20)
project(f401_bare C ASM)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

set(CMAKE_C_STANDARD 11)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CPU_FLAGS
  -mcpu=cortex-m4
  -mthumb
  -mfpu=fpv4-sp-d16
  -mfloat-abi=hard
)

set(OPT_FLAGS
  -ffunction-sections
  -fdata-sections
)

set(WARN_FLAGS
  -Wall
  -Wextra
  -Werror=return-type
  -Wno-unused-parameter
)

set(DEBUG_FLAGS
  -Og
  -g3
  -ggdb
  -fno-lto
)

set(RELEASE_FLAGS
  -O2
)

set(SRCS
  src/main.c
  src/system_stm32f4xx.c
  mcu_core/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
  mcu_core/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
  mcu_core/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
)

list(APPEND SRCS ${FREERTOS_KERNEL})

set(STARTUP  ${CMAKE_CURRENT_SOURCE_DIR}/mcu_core/startup/startup_stm32f401xe.s)
set(LDSCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/mcu_core/ld/STM32F401XE.ld)

add_executable(${PROJECT_NAME}.elf ${SRCS} ${STARTUP})

target_include_directories(${PROJECT_NAME}.elf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/mcu_core/CMSIS/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/mcu_core/CMSIS/Device/ST/STM32F4xx/Include
  ${CMAKE_CURRENT_SOURCE_DIR}/mcu_core/STM32F4xx_StdPeriph_Driver/inc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/common
  ${CMAKE_CURRENT_SOURCE_DIR}/src/app
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/src/compiler/gcc
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
  STM32F401xx
  USE_STDPERIPH_DRIVER
)

target_compile_options(${PROJECT_NAME}.elf PRIVATE
  ${CPU_FLAGS} ${OPT_FLAGS} ${WARN_FLAGS}
  $<$<CONFIG:Debug>:${DEBUG_FLAGS}>
  $<$<CONFIG:Release>:${RELEASE_FLAGS}>
)

target_link_options(${PROJECT_NAME}.elf PRIVATE
  ${CPU_FLAGS} ${OPT_FLAGS}
  -T${LDSCRIPT}
  -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map
  $<$<CONFIG:Debug>:-g;-fno-lto>
  $<$<CONFIG:Release>:${RELEASE_FLAGS}>
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex  $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
  COMMENT "Generating HEX, BIN and size"
)