#include "../libopencm3/include/libopencm3/stm32/adc.h"
#include "../libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libopencm3/include/libopencm3/stm32/gpio.h"
#include "../libopencm3/include/libopencm3/stm32/usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

uint8_t channel_array[] = {1, 1, ADC_CHANNEL_TEMP};

static void adc_setup(void)
{
	rcc_periph_clock_enable(RCC_ADC);
	rcc_periph_clock_enable(RCC_GPIOA);

	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_enable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_set_resolution(ADC1, ADC_RESOLUTION_10BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
	{ /* Wait a bit. */
		__asm__("nop");
	}
}

static void usart_setup(void)
{
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

int main(void)
{
	char buffer[12];
	uint16_t raw;
	adc_setup();
	usart_setup();

	while (1)
	{
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)))
			;

		raw = adc_read_regular(ADC1);

		double voltage = raw * (3.0 / 1023);

		snprintf(buffer, sizeof(buffer), "%.04f", voltage);

		char msg[] = "Reading voltage from 3.0v pin: ";

		int sizeMSG = strlen(msg);
		for (int i = 0; i < sizeMSG; i++)
			usart_send_blocking(USART1, msg[i]);
		int sizeBUF = strlen(buffer);
		for (int i = 0; i < sizeBUF; i++)
			usart_send_blocking(USART1, buffer[i]);

		usart_send_blocking(USART1, '\r');
		usart_send_blocking(USART1, '\n');

		int i;
		for (i = 0; i < 800000; i++)
		{ /* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;
}