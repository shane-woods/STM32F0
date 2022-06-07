#include "../libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libopencm3/include/libopencm3/stm32/adc.h"
#include "../libopencm3/include/libopencm3/stm32/usart.h"
#include "../libopencm3/include/libopencm3/stm32/gpio.h"
#include "../libopencm3/include/libopencm3/stm32/spi.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


static void clock_setup(void)
{
    /* Enable clocks for GPIO port A */
    rcc_periph_clock_enable(RCC_GPIOA);
    
    /* Enable clocks for GPIO port B */
    rcc_periph_clock_enable(RCC_GPIOB);        

    /* Enable SPI1 Periph and gpio clocks */
    rcc_periph_clock_enable(RCC_SPI1);
}
static void spi_setup(void)
{
    /* Configure GPIOs: SS=PA4, SCK=PA5, MISO=PA6 and MOSI=PA7 */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);

    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6);
	gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO5 | GPIO6);

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI1);

    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/64 of peripheral clock frequency
     * Clock polarity: Idle High
     * Clock phase: Data valid on 2nd clock pulse
     * Data frame format: 8-bit
     * Frame format: MSB First
     */
    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_MSBFIRST);
    
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
#define PORT_LED GPIOC
#define PIN_LED GPIO8
static void gpio_setup(void)
{
    /* Enable GPIOC clock. */
    /* Manually: */
    // RCC_AHBENR |= RCC_AHBENR_GPIOCEN;
    /* Using API functions: */
    rcc_periph_clock_enable(RCC_GPIOC);
    /* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
    /* Using API functions: */
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED);
}
int main(void)
{
    clock_setup();
    spi_setup();
    usart_setup();
    gpio_setup();
    /*
    Global variables
    */
    uint16_t raw;
    double voltage;
    char uart_buf[20];
    int uart_buf_len;
    char raw_buf[50];
    int raw_buf_len;
    char voltage_buf[50];
    int voltage_buf_len;
    int i = 0;

    while (1) {

        // Transmit UART to verify everything is okay
        snprintf(uart_buf, sizeof(uart_buf), "SPI Test %d", i);
        uart_buf_len = strlen(uart_buf);
        for (int j = 0; j < uart_buf_len; j++)
            usart_send_blocking(USART1, uart_buf[j]);
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');

        gpio_clear(GPIOB, GPIO6);

        /* This should set the CNV pin high and therfore start the conversion */
        gpio_set(GPIOB, GPIO6);

        gpio_clear(GPIOB, GPIO6);

        /* Send a dummy byte because we just need to read from ADC */
        spi_send(SPI1, 0x00);

        /* Read a byte from ADC */

        raw = spi_read(SPI1);

        spi_send(SPI1, 0x00);

        i++;

        voltage = raw * (5.0/65535); 

        snprintf(raw_buf, sizeof(raw_buf), "Raw digital value from ADC: %d", raw);
        raw_buf_len = strlen(raw_buf);

        for (int j = 0; j < raw_buf_len; j++) 
            usart_send_blocking(USART1, raw_buf[j]);

        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');
        
        snprintf(voltage_buf, sizeof(voltage_buf), "Voltage from ADC: %.02f", voltage);
        voltage_buf_len = strlen(voltage_buf);

        for (int j = 0; j < voltage_buf_len; j++)
            usart_send_blocking(USART1, voltage_buf[j]);
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');
        usart_send_blocking(USART1, '\n');

        gpio_toggle(PORT_LED, PIN_LED);

        for (int j = 0; j < 800000; j++)
        { /* Wait a bit. */
            __asm__("nop");
        }
    }
}

