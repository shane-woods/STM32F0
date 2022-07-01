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

/* Buffer that will store the 8 byte packets */
int uartbuf[64];

/** Queue indexes for the buffer?
 * We beleive this might be like a circular
 * queue but don't know
 */
int qnext, qlast;

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

/**
 * @brief
 * Buffered single char output (8 bits).
 * Put in mem buf and return. ISR sends it
 *
 * @param ch
 */
void putch(int ch)
{
  usart_disable_tx_interrupt(USART1); /* prevent collision ISR access to qlast */
  uartbuf[qlast++] = ch;
  qlast &= 0x3f;
  usart_enable_tx_interrupt(USART1); /* What do these do? */
}

/**
 * @brief write a 16-bit (word) value to UART
 *
 * @param wd
 */
void putwd(long wd)
{
  /* Put MSB in first */
  putch((wd & 0xFF00) >> 8);

  /* Put LSB in second */
  putch((wd & 0xFF));

  /* I might want to change these to like their own LSB and MSB variables */
}

/**
 * @brief write a 16-bit (word) value swapped to UART
 *
 * @param wd
 */
void putswap(long wd)
{
  /* Put LSB in first */
  putch((wd & 0xFF));

  /* Put MSB in second */
  putch((wd & 0xFF00) >> 8);

  char wd_buf[50];
  int wd_buf_len = snprintf(wd_buf, sizeof(wd_buf), "Raw %d", wd);
  for (int i = 0; i < wd_buf_len; i++)
    usart_send_blocking(USART1, wd_buf[i]);
  usart_send_blocking(USART1, '\r');
  usart_send_blocking(USART1, '\n');

  /* I might want to change these to like their own LSB and MSB variables */
}

void setupUART(void)
{
  /* Setup UART parameters. */

  /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
  gpio_mode_setup(USART_TX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_GPIO_PIN);
  gpio_set_af(USART_TX_GPIO_PORT, GPIO_AF1, USART_TX_GPIO_PIN);

  /* Enables global interrupts */
  nvic_enable_irq(NVIC_USART1_IRQ);

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