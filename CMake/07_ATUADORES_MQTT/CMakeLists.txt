cmake_minimum_required(VERSION 3.12)    #Versão mínima requerida

include(pico_sdk_import.cmake)          #Configura as variáveis de ambiente pro SDK da RPPi

project(MOTOR_MQTT C CXX ASM)            #Nome do projeto e linguagens suportadas

set(CMAKE_C_STANDARD 11)                #Versão 11 de C
set(CMAKE_CXX_STANDARD 17)              #Versão 17 do C++

set(PICO_BOARD pico_w)                  #Tipo da placa

pico_sdk_init()                         #Configura as variáveis de ambiente

add_executable(MOTOR_MQTT main.c)        #Gera o executável

pico_enable_stdio_usb(MOTOR_MQTT 1)      #Habilita a entrada e saída padrão pela USB
pico_enable_stdio_uart(MOTOR_MQTT 0)     #Desabilita a entrada UART

#Incluindo arquivos .h para essa aplicação em específico
target_include_directories(MOTOR_MQTT PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}       
)
#Bibliotecas lincadas
target_link_libraries(MOTOR_MQTT
        tinyusb_board
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib
        pico_lwip_mqtt
        hardware_adc
        hardware_pwm
    )

pico_add_extra_outputs(MOTOR_MQTT)       #Gera o .uf2
