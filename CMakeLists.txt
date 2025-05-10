# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

cmake_minimum_required(VERSION 3.13)  # Padrões
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Definindo a placa
set(PICO_BOARD pico_w CACHE STRING "Board type")

# SDK do Pico
include(pico_sdk_import.cmake)

# Caminho para o FreeRTOS (corrigido)
set(FREERTOS_KERNEL_PATH "C:/Users/Jonas/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Nome do projeto e executável
project(wifi_project C CXX ASM)
pico_sdk_init()

# Diretórios de include do projeto
include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR})

# Criando o executável
add_executable(wifi_project
    main.c
    lib/Display_Bibliotecas/ssd1306.c
    lib/Matriz_Bibliotecas/matriz_led.c
    lib/DS18b20/ds18b20.c
    
)

# Bibliotecas utilizadas
target_link_libraries(wifi_project
    pico_stdlib
    hardware_i2c       # <-- driver I²C do pico-sdk
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_pwm
    hardware_pio
    pico_cyw43_arch_lwip_threadsafe_background
    hardware_adc
)

# Habilita I/O padrão via USB (ou UART)
pico_enable_stdio_usb(wifi_project 1)
pico_enable_stdio_uart(wifi_project 1)

# Gera arquivos binários/UF2
pico_add_extra_outputs(wifi_project)
