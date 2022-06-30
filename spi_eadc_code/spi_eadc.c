/**
 * @file spi_eadc.c
 * @author Shane Wooods and Jared King
 * @brief Code to read voltage from a 16-bit external ADC (AD7980)
 * @version 0.1
 * @date 2022-06-13
 *
 * IC pin Configuration for AD7980
 *
 *               AD7980 +--------+ AD7980
 *  +5v   <----> Vref 1 |        | 10 VIO <---> +3.3v
 *  +2.5v <----> Vdd  2 |        | 9  SDI <---> VIO
 *  +2.5v <----> IN+  3 |        | 8  SCK <---> STM32 PA5 (CLOCK)
 *  GND   <----> IN-  4 |        | 7  SDO <---> STM32 PA6 (MOSI)
 *  GND   <----> GND  5 |        | 6  CNV <---> STM32 PA8 (TIM1)
 *                      +--------+
 */

#ifndef STM32F0
#define STM32F0
#endif

#include "spi_eadc.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static void clock_setup(void)
{

    /* Enable clock at 48mhz */
    rcc_clock_setup_in_hsi_out_48mhz();

    /* Enables clocks for all needed peripherals */
    rcc_periph_clock_enable(GPIO_CLOCKS);
    rcc_periph_clock_enable(SPI1_CLOCK);
    rcc_periph_clock_enable(USART1_CLOCK);
    rcc_periph_clock_enable(TIMER1_CLOCK);

    /* Not using these just yet */
    rcc_periph_clock_enable(IADC_CLOCK);
    rcc_periph_clock_enable(DAC_CLOCK);
}

static void spi_setup(void)
{

    /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
    spi_reset(SPI1);

    /* Set up SPI in Master mode with:
     * Clock baud rate: 1/4 of peripheral clock frequency
     * Clock polarity: Idle High
     * Clock phase: Data valid on 1st clock pulse
     * Data frame format: 8-bit
     * Frame format: MSB First
     */
    spi_init_master(SPI1, SPI_BUADRATE_PRESCALER, SPI_CLOCK_POLARITY,
                    SPI_CPHA_CLOCK_TRANSITION, SPI_MSB_FIRST);

    /* AD7980 is 16-bit resolution, need to set data size to 16-bits */
    spi_set_data_size(SPI1, 16);

    /*
     * Set NSS management to software.
     *
     * Note:
     * Setting nss high is very important, even if we are controlling the GPIO
     * ourselves this bit needs to be at lea st set to 1, otherwise the spi
     * peripheral will not send any data out.
     */
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    /* Enable SPI1 periph. */
    spi_enable(SPI1);
}

static void timer_setup(void)
{

    /* Disable/Reset Timer? */
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP); // FIXME THIS IS NEW

    /* Enables TIM1_CH1 in Output Compare Mode */
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FORCE_HIGH);
    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_break_main_output(TIM1);

    /* I believe this is the value at which the timer interrupt will occur */
    timer_set_oc_value(TIM1, TIM_OC1, 24000);

    /**
     * Prescaler divides the clock counter
     * Not sure what we will eventually need it at
     */
    timer_set_prescaler(TIM1, 480 - 1);

    /* So we will eventually want this to be 125ms */
    timer_set_period(TIM1, 48000 - 1);

    /* Enable Timer to generate interrupt signal */
    timer_generate_event(TIM1, TIM_EGR_CC1G | TIM_EGR_TG);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);

    /* Start the timer */
    TIM_CR1(TIM1) |= TIM_CR1_CEN;
}

static void usart_setup(void)
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

static void gpio_setup(void)
{
    /** LED GPIO SETUP **/
    /* Set GPIO8 (in GPIO port C) to 'output push-pull'. */
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BLUE_LED_PIN | GREEN_LED_PIN);

    /** SPI GPIO SETUP **/
    /* Configure CNV GPIO as PA8 (TIM1_CH1) */
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

void tim1_cc_isr(void)
{

    int raw; /* 32 bit value */
    char raw_buf[50];
    int raw_buf_len;

    /* Toggle Blue LED on interrupt */
    gpio_toggle(LED_PORT, BLUE_LED_PIN);

    /* Send byte to initialize SPI transfer */
    SPI1_DR = 0x1;
    while (!(SPI1_SR & SPI_SR_RXNE))
        ; /* wait for SPI transfer complete */

    /* Get raw adc value from data register */
    raw = SPI1_DR;

    uint8_t MSB = ((raw & 0xFF00) >> 8); /* MSB HERE */
    uint8_t LSB = (raw & 0xFF);          /* LSB HERE */
    raw = 0x0;
    raw = (raw | LSB);
    raw = raw << 8;
    raw = raw | MSB;

    raw_buf_len = snprintf(raw_buf, sizeof(raw_buf), "Raw %d", raw);
    for (int i = 0; i < raw_buf_len; i++)
        usart_send_blocking(USART1, raw_buf[i]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    /**
     * Equation to calculate voltage
     * Voltage = Raw * (5 / 65535)
     *
     */

    /* extern ADC readout completed. Force CNV low */
    // TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW; // (assumes all other bits are zero)
    TIM1_CCMR1 = TIM_CCMR1_OC1M_TOGGLE;

    TIM1_SR = ~TIM_SR_CC1IF; // clear interrupt
}

int main(void)
{
    clock_setup();
    spi_setup();
    gpio_setup();
    usart_setup();
    timer_setup();

    while (1)
        ;
}
