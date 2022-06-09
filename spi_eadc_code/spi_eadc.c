#ifndef STM32F0
#define STM32F0
#endif

#include "../libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libopencm3/include/libopencm3/stm32/adc.h"
#include "../libopencm3/include/libopencm3/stm32/usart.h"
#include "../libopencm3/include/libopencm3/stm32/gpio.h"
#include "../libopencm3/include/libopencm3/stm32/spi.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/** @defgroup RCC Clock handles **/
#define GPIO_CLOCKS RCC_GPIOA | RCC_GPIOB | RCC_GPIOC /* clocks for GPIO ports A & B */
#define USART1_CLOCK RCC_USART1                       /* clock for USART1 */
#define SPI1_CLOCK RCC_SPI1                           /* clock for SPI1 interface to 16-bit external ADC (AD7980) */
#define TIMER1_CLOCK RCC_TIM1                         /* CNV clock pulse to external ADC (AD7980) */
#define IADC_CLOCK RCC_ADC                            /* clocks for internal Housekeeping ADCs */
#define DAC_CLOCK RCC_DAC                             /* clocks for DACs (I beleive this is for sweeping, haven't used yet) */

/** @defgroup SPI Handles **/
#define SPI_BUADRATE_PRESCALER SPI_CR1_BAUDRATE_FPCLK_DIV_32
#define SPI_CLOCK_POLARITY_1 SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE
#define SPI_CPHA_CLOCK_TRANSITION_2 SPI_CR1_CPHA_CLK_TRANSITION_2
#define SPI_MSB_FIRST SPI_CR1_MSBFIRST

/** @defgroup USART Handles **/
#define USART_TX_GPIO_PORT GPIOA
#define USART_TX_GPIO_PIN GPIO9
#define TX_MODE USART_MODE_TX

/** @defgroup GPIO Handles for on STM LEDs **/
#define LED_PORT GPIOC
#define BLUE_LED_PIN GPIO8
#define GREEN_LED_PIN GPIO9

/** @defgroup GPIO Handles for SPI **/
#define SCK_GPIO_PORT GPIOA
#define SCK_GPIO_PIN GPIO5
#define MISO_GPIO_PORT GPIOA
#define MISO_GPIO_PIN GPIO6
#define CNV_GPIO_PORT GPIOB
#define CNV_GPIO_PIN GPIO6

static void clock_setup(void)
{

    /* Enable clock at 48mhz */
    rcc_clock_setup_in_hsi_out_48mhz();

    rcc_periph_clock_enable(GPIO_CLOCKS);
    rcc_periph_clock_enable(SPI1_CLOCK);
    rcc_periph_clock_enable(USART1_CLOCK);
    rcc_periph_clock_enable(TIMER1_CLOCK);
    rcc_periph_clock_enable(IADC_CLOCK);
    rcc_periph_clock_enable(DAC_CLOCK);
}

static void spi_setup(void)
{

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI1);

    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/64 of peripheral clock frequency
     * Clock polarity: Idle High
     * Clock phase: Data valid on 2nd clock pulse
     * Data frame format: 8-bit
     * Frame format: MSB First
     */
    spi_init_master(SPI1, SPI_BUADRATE_PRESCALER, SPI_CLOCK_POLARITY_1,
                    SPI_CPHA_CLOCK_TRANSITION_2, SPI_MSB_FIRST);

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

static void timer_setup(void)
{
}

static void usart_setup(void)
{
    /* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(USART_TX_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_GPIO_PIN);
    gpio_set_af(USART_TX_GPIO_PORT, GPIO_AF1, USART_TX_GPIO_PIN);

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

static void gpio_setup(void)
{
    /** LED GPIO SETUP **/
    /* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BLUE_LED_PIN | GREEN_LED_PIN);

    /** SPI GPIO SETUP **/
    /* Configure CNV GPIO as PB6 */
    gpio_mode_setup(CNV_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CNV_GPIO_PIN);
    gpio_set_af(CNV_GPIO_PORT, GPIO_AF2, CNV_GPIO_PIN);
    gpio_set_output_options(CNV_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, CNV_GPIO_PIN);

    /* Configure SCK GPIO as PA5 */
    gpio_mode_setup(SCK_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SCK_GPIO_PIN);
    gpio_set_af(SCK_GPIO_PORT, GPIO_AF0, SCK_GPIO_PIN);
    gpio_set_output_options(SCK_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, SCK_GPIO_PIN);

    /* Configure MISO GPIO as PA6 */
    gpio_mode_setup(MISO_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, MISO_GPIO_PIN);
    gpio_set_af(MISO_GPIO_PORT, GPIO_AF0, MISO_GPIO_PIN);
    gpio_set_output_options(MISO_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, MISO_GPIO_PIN);
}

int main(void)
{
    clock_setup();
    spi_setup();
    usart_setup();
    gpio_setup();

    /* Global variables */
    uint16_t raw;
    double voltage;
    char uart_buf[20];
    int uart_buf_len;
    char raw_buf[50];
    int raw_buf_len;
    char voltage_buf[50];
    int voltage_buf_len;
    int i = 0;

    while (1)
    {

        // Transmit UART to verify everything is okay
        snprintf(uart_buf, sizeof(uart_buf), "SPI Test %d", i);
        uart_buf_len = strlen(uart_buf);
        for (int j = 0; j < uart_buf_len; j++)
            usart_send_blocking(USART1, uart_buf[j]);
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');

        /* This should set the CNV pin high and therfore start the conversion */
        gpio_set(GPIOB, GPIO6);

        for (int j = 0; j < 800000; j++)
        { /* Wait a bit. */
            __asm__("nop");
        }

        gpio_clear(GPIOB, GPIO6);

        /* Send a dummy byte because we just need to read from ADC */
        spi_send(SPI1, 0x00);

        /* Read a byte from ADC */
        raw = spi_read(SPI1);

        i++;

        voltage = raw * (5.0 / 65535);

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

        gpio_toggle(LED_PORT, BLUE_LED_PIN);
        for (int j = 0; j < 800000; j++)
        { /* Wait a bit. */
            __asm__("nop");
        }
        gpio_toggle(LED_PORT, GREEN_LED_PIN);
    }
}
