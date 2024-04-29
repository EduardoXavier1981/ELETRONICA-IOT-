#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "bsp/board.h"

#define EN_A 18
#define A1 16
#define A2 17

int main() {

    gpio_init(EN_A);
    gpio_init(A1);
    gpio_init(A2);

    gpio_set_dir(EN_A, GPIO_OUT);
    gpio_set_dir(A1, GPIO_OUT);
    gpio_set_dir(A2, GPIO_OUT);

    while(1) {
        if(board_button_read()){
            gpio_put(EN_A, false);
            sleep_ms(20);
            gpio_put(A1, false);
            gpio_put(A2, true);
            gpio_put(EN_A, true);
        } else {
            gpio_put(EN_A, false);
            sleep_ms(20);
            gpio_put(A1, true);
            gpio_put(A2, false);
            gpio_put(EN_A, true);
        }
        sleep_ms(200);
    }
}