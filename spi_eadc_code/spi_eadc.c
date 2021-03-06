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
#include "uart.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// output buffer
int uartbuf[64];
long qnext, qlast;

// Buffered single char output. Put in mem buf and return. ISR sends it
void putch(int ch)
{
    usart_disable_tx_interrupt(USART1); // prevent collision ISR access to qlast
    uartbuf[qlast++] = ch;
    qlast &= 0x3f;
    usart_enable_tx_interrupt(USART1);
}

void putwd(long wd) // Write 16-bit value to UART
{
    putch((wd & 0xFF00) >> 8); // msb
    putch(wd & 0xFF);          // lsb
}

void putswab(long wd) // Write 16-bit byte swappped to UART
{
    putch(wd & 0xFF);          // lsb
    putch((wd & 0xFF00) >> 8); // msb
}

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

static void usart_setup(void)
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

    qnext = 0;
    qlast = 0;

    /* Finally enable the USART. */
    usart_enable(USART1);
    putch('!'); // send a char to show it was reset
}

// Set up for single chan software trig conversion.
// Assumes GPIO bits are already set up
void hk_setup(void)
{
    adc_power_off(ADC1);
    adc_calibrate(ADC1); // requires adc disabled
    while (adc_is_calibrating(ADC1))
        ;

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_055DOT5); // works best for SWPMON
    adc_power_on(ADC1);

    adc_enable_temperature_sensor();
    adc_enable_vrefint();

    adc_set_clk_source(ADC1, ADC_CLKSOURCE_PCLK_DIV2); // use sync clock for no jitter
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_set_right_aligned(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_single_conversion_mode(ADC1);
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
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP); // FIXME THIS IS NEW

    /* Enables TIM1_CH1 in Output Compare Mode */
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_TOGGLE);
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

void tim1_cc_isr(void)
{

    /* Toggle Blue LED on interrupt */
    gpio_toggle(LED_PORT, GREEN_LED_PIN);

    /* Send byte to initialize SPI transfer */
    SPI1_DR = 0x1;
    while (!(SPI1_SR & SPI_SR_RXNE))
        ; /* wait for SPI transfer complete */

    /* Get raw adc value from data register */
    int raw = SPI1_DR;

    char wd_buf[50];
    int wd_buf_len = snprintf(wd_buf, sizeof(wd_buf), "Raw %d", raw);
    for (int i = 0; i < wd_buf_len; i++)
        usart_send_blocking(USART1, wd_buf[i]);
    usart_send_blocking(USART1, '\r');
    usart_send_blocking(USART1, '\n');

    /* Send RPA sample (SPI readout has bytes swapped) */
    putswap(raw);

    /* THIS SENDS GOOD BITS, BUT DOESNT SHOW ON SCREEN */
    // putswab(raw); // Send RPA sample (SPI readout has bytes swapped)

    /**
     * Equation to calculate voltage
     * Voltage = Raw * (5 / 65535)
     */
    /* extern ADC readout completed. Force CNV low */
    TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_HIGH; // (assumes all other bits are zero)
    TIM1_CCMR1 = TIM_CCMR1_OC1M_TOGGLE;

    TIM1_SR = ~TIM_SR_CC1IF; // clear interrupt
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

int main(void)
{
    setupUART();
    clock_setup();
    gpio_setup();
    spi_setup();
    timer_setup();
    while (1)
        ;
}