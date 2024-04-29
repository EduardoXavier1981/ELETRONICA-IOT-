#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "bsp/board.h"

#define EN_A 18
#define A1 16
#define A2 17

int main() {

    gpio_set_function(EN_A, GPIO_FUNC_PWM);
    int fatia_pwm = pwm_gpio_to_slice_num(EN_A);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.0f);
    pwm_init(fatia_pwm, &config, true);

    gpio_init(A1);
    gpio_init(A2);

    gpio_set_dir(A1, GPIO_OUT);
    gpio_set_dir(A2, GPIO_OUT);

    gpio_put(A1, true);
    gpio_put(A2, false);
    //100% do motor = 65535
    pwm_set_gpio_level(EN_A, 65535);
    bool subir = true;
    uint16_t duty_cycle = 0;

    while(1) {
        if (subir) {
            if(board_button_read()){
                duty_cycle += 1000;
            }
            if (duty_cycle >= 65535){
                subir = false;
            }
        } else {
            if(board_button_read()){
                duty_cycle -= 1000;
            }
            if (duty_cycle <= 1000){
                subir = true;
            }
        }
        
        pwm_set_gpio_level(EN_A, duty_cycle);
        sleep_ms(200);
    }
}