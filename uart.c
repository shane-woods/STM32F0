/**
 * @file uart.c
 * @author Shane Woods & Jared King
 * @brief
 * This file will be used to transmit 8 byte
 * (4 16-bit words) data packets from the MCU to
 * the OBC
 * @version 0.1
 * @date 2022-06-29
 */

#include "uart.h"

#include <stdio.h>
#include <string.h>

#include "libopencm3/include/libopencm3/stm32/rcc.h" /* Fine to be here now but Clock setup can go in our main later */
#include "libopencm3/include/libopencm3/stm32/usart.h"
#include "libopencm3/include/libopencm3/stm32/gpio.h" /* Fine for LEDs, GPIO setup will go in main later */
#include "libopencm3/include/libopencm3/cm3/nvic.h"

int uartbuf[64];
int qnext, qlast;

/* Buffered single char output (8 bits). Put in mem buf and return. ISR sends it */
void putch(int ch)
{
    usart_disable_tx_interrupt(USART1); // prevent collision ISR access to qlast
    uartbuf[qlast++] = ch;
    qlast &= 0x3f;
    usart_enable_tx_interrupt(USART1);
}

/**
 * @brief write a 16-bit (word) value to UART
 *
 * @param wd
 */
void putwd(long wd)
{
    putch((wd & 0xFF00) >> 8); // msb
    putch(wd & 0xFF);          // lsb
}

void putswap(long wd)
{
    putch(wd & 0xFF);          // lsb
    putch((wd & 0xFF00) >> 8); // msb
}

void setupUART(void)
{
    nvic_enable_irq(NVIC_USART1_IRQ);

    /* Setup UART parameters. */

    /* Baudrate at 115200 bits/sec */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, 1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);

    /* Finally enable the USART. */
    usart_enable(USART1);
    putch('!'); // send a char to show it was reset
}
void usart1_isr(void)
{
    if (USART_ISR(USART1) & USART_ISR_TXE) // (should be the only bit enabled)
    {
        if (qnext != qlast)
        {
            gpio_set(GPIOC, GPIO9);
            USART_TDR(USART1) = uartbuf[qnext++]; // this prints a few bad characters
        }
        else
        {
            usart_disable_tx_interrupt(USART1);
            gpio_clear(GPIOC, GPIO9);
        }
        qnext &= 0x3f;
    }
}