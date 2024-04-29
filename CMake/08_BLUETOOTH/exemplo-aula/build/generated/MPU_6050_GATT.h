
// C:/Users/Acer/Desktop/Code/Raspberry/CMake/07_BLUETOOH/exemplo-aula/build/generated/MPU_6050_GATT.h generated from C:/Users/Acer/Desktop/Code/Raspberry/CMake/07_BLUETOOH/exemplo-aula/MPU_6050_GATT.gatt for BTstack
// it needs to be regenerated when the .gatt file is updated. 

// To generate C:/Users/Acer/Desktop/Code/Raspberry/CMake/07_BLUETOOH/exemplo-aula/build/generated/MPU_6050_GATT.h:
// C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk/lib/btstack/tool/compile_gatt.py C:/Users/Acer/Desktop/Code/Raspberry/CMake/07_BLUETOOH/exemplo-aula/MPU_6050_GATT.gatt C:/Users/Acer/Desktop/Code/Raspberry/CMake/07_BLUETOOH/exemplo-aula/build/generated/MPU_6050_GATT.h

// att db format version 1

// binary attribute representation:
// - size in bytes (16), flags(16), handle (16), uuid (16/128), value(...)

#include <stdint.h>

// Reference: https://en.cppreference.com/w/cpp/feature_test
#if __cplusplus >= 200704L
constexpr
#endif
const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x1a, 0x18, 
    // 0x0002 CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE - READ | NOTIFY | DYNAMIC
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x12, 0x03, 0x00, 0x6e, 0x2a, 
    // 0x0003 VALUE CHARACTERISTIC-ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE - READ | NOTIFY | DYNAMIC
    // READ_ANYBODY
    0x08, 0x00, 0x02, 0x01, 0x03, 0x00, 0x6e, 0x2a, 
    // 0x0004 CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x04, 0x00, 0x02, 0x29, 0x00, 0x00, 
    // END
    0x00, 0x00, 
}; // total size 25 bytes 


//
// list service handle ranges
//
#define ATT_SERVICE_ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING_START_HANDLE 0x0001
#define ATT_SERVICE_ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING_END_HANDLE 0x0004
#define ATT_SERVICE_ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING_01_START_HANDLE 0x0001
#define ATT_SERVICE_ORG_BLUETOOTH_SERVICE_ENVIRONMENTAL_SENSING_01_END_HANDLE 0x0004

//
// list mapping between characteristics and handles
//
#define ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_ORG_BLUETOOTH_CHARACTERISTIC_TEMPERATURE_01_CLIENT_CONFIGURATION_HANDLE 0x0004
