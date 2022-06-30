#include "../libopencm3/include/libopencm3/stm32/rcc.h"
#include "../libopencm3/include/libopencm3/stm32/adc.h"
#include "../libopencm3/include/libopencm3/stm32/usart.h"
#include "../libopencm3/include/libopencm3/stm32/gpio.h"
#include "../libopencm3/include/libopencm3/stm32/spi.h"
#include "../libopencm3/include/libopencm3/stm32/timer.h"
#include "../libopencm3/include/libopencm3/cm3/nvic.h"

/** @defgroup RCC Clock handles **/
#define GPIO_CLOCKS RCC_GPIOA | RCC_GPIOB | RCC_GPIOC /* clocks for GPIO ports A & B */
#define USART1_CLOCK RCC_USART1                       /* clock for USART1 */
#define SPI1_CLOCK RCC_SPI1                           /* clock for SPI1 interface to 16-bit external ADC (AD7980) */
#define TIMER1_CLOCK RCC_TIM1                         /* CNV clock pulse to external ADC (AD7980) */
#define IADC_CLOCK RCC_ADC                            /* clocks for internal Housekeeping ADCs */
#define DAC_CLOCK RCC_DAC                             /* clocks for DACs (I beleive this is for sweeping, haven't used yet) */

/** @defgroup SPI Handles **/
#define SPI_BUADRATE_PRESCALER SPI_CR1_BAUDRATE_FPCLK_DIV_4
#define SPI_CLOCK_POLARITY SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
#define SPI_CPHA_CLOCK_TRANSITION SPI_CR1_CPHA_CLK_TRANSITION_2
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
#define MOSI_GPIO_PORT GPIOA
#define MOSI_GPIO_PIN GPIO7
#define CNV_GPIO_PORT GPIOA
#define CNV_GPIO_PIN GPIO8

static void gpio_setup(void);
static void usart_setup(void);
static void timer_setup(void);
static void spi_setup(void);
static void clock_setup(void);
