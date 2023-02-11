/**
 * @file test_00_uart.c
 * @author cy023
 * @date 2023.02.08
 * @brief 
 * 
 */

#include <stdio.h>
#include "system.h"
#include "systick.h"
#include "delay.h"
#include "uart.h"

int main()
{
    system_init();
    printf("System Boot.\n");
    printf("[test00]: uart ...\n");

    char c;
    while (1) {
        printf("Please input a character: \n");
        c = uart0_getc();
        // scanf("%c", &c); // FIXME:
        printf("Your input character is %c\n\n", c);
    }
    return 0;
}
