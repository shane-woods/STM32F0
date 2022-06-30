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

int uartbuf[64];
int qnext, qlast;

/* Buffered single char output (8 bits). Put in mem buf and return. ISR sends it */
void putch(int ch)
{
}

/**
 * @brief write a 16-bit (word) value to UART
 *
 * @param wd
 */
void putwd(long wd)
{
}

void putswap(long wd)
{
}

void setupUART(void)
{
  /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
  gpio_mode_setup(USART_TX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_GPIO_PIN);
  gpio_set_af(USART_TX_GPIO_PORT, GPIO_AF1, USART_TX_GPIO_PIN);

  /* Setup UART parameters. */

  /* Baudrate at 115200 bits/sec */
  usart_set_baudrate(USART1, 115200);

  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
}