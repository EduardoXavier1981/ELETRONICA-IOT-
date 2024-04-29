#include <stdio.h>
#include "pico/stdlib.h"

int main()
{
    stdio_init_all();
    while (1)
    {
        printf("Testando Serial Print\n");
        sleep_ms(1000);
    }
}