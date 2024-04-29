#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
 
#define BUILTIN_LED CYW43_WL_GPIO_LED_PIN
 
int main() {
    stdio_init_all();
    if (cyw43_arch_init()) {
        return -1;
    }
    while (1) {
        cyw43_arch_gpio_put(BUILTIN_LED, 1);
        sleep_ms(100);
        cyw43_arch_gpio_put(BUILTIN_LED, 0);
        sleep_ms(100);
    }
}