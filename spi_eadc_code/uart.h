/**
 * @file uart.h
 * @author Shane Woods & Jared King
 * @brief
 * Header files
 * @version 0.1
 * @date 2022-06-30
 */

#ifndef STM32F0
#define STM32F0
#endif

#include "../libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libopencm3/include/libopencm3/stm32/adc.h"
#include "../libopencm3/include/libopencm3/stm32/usart.h"
#include "../libopencm3/include/libopencm3/stm32/gpio.h"
#include "../libopencm3/include/libopencm3/stm32/spi.h"
#include "../libopencm3/include/libopencm3/stm32/timer.h"
#include "../libopencm3/include/libopencm3/cm3/nvic.h"

/** @defgroup USART Handles **/
#define USART_TX_GPIO_PORT GPIOA
#define USART_TX_GPIO_PIN GPIO9
#define TX_MODE USART_MODE_TX

void setupUART(void);
void putch(int ch);
void putwd(long wd);
void putswap(long wd);