#include "libopencm3/include/libopencm3/stm32/rcc.h"
#include "libopencm3/include/libopencm3/stm32/adc.h"
#include "libopencm3/include/libopencm3/stm32/usart.h"
#include "libopencm3/include/libopencm3/stm32/gpio.h"
#include "libopencm3/include/libopencm3/stm32/spi.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void clock_setup(void)
{
	/* Enable GPIOA clock. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable clocks for GPIO port A */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable SPI1 Periph and gpio clocks */
	rcc_periph_clock_enable(RCC_SPI1);
}

static void spi_setup(void) {
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5 | GPIO7);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO6);

	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                  SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_MSBFIRST);

	spi_reset(SPI1);
	/*
   	* Set NSS management to software.
   	*
	* Note:
   	* Setting nss high is very important, even if we are controlling the GPIO
   	* ourselves this bit needs to be at least set to 1, otherwise the spi
   	* peripheral will not send any data out.
   	*/
  	spi_enable_software_slave_management(SPI1);
  	spi_set_nss_high(SPI1);

  	/* Enable SPI1 periph. */
  	spi_enable(SPI1);

}

static void usart_setup(void) {
    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART1. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF1, GPIO9);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

int main(void) {
	clock_setup();
	spi_setup();
	usart_setup();

	/*
	Global variables
	*/
	uint16_t raw;
	float voltage;
	char uart_buf[20];
	int uart_buf_len;
	int raw_buf_len;
	char raw_buf[20];
	char voltage_buf[20];
	int voltage_buf_len;

	int i = 0;
    while (1) {
        // Transmit UART to verify everything is okay
        snprintf(uart_buf, sizeof(uart_buf), "SPI Test %d\r\n", i);
		uart_buf_len = strlen(uart_buf);
		for (int j = 0; j < uart_buf_len; j++) 
			usart_send_blocking(USART1, uart_buf[j]);
		
        i++;

        // Read status register
        raw = spi_read8(SPI1);
      
        //Calculate voltage
        voltage = 65536 / (raw * 5);

        //Print voltage
       	snprintf(voltage_buf, sizeof(voltage_buf), "Voltage from ADC: %f\r\n", voltage);
		voltage_buf_len = strlen(voltage_buf); 
		for (int j = 0; j < voltage_buf_len; j++)
			usart_send_blocking(USART1, voltage_buf[j]);

		usart_send_blocking(USART1, '\r');
		usart_send_blocking(USART1,'\n');

        for (int j = 0; j < 800000; j++) {   /* Wait a bit. */
			__asm__("nop");
		}
    }


}