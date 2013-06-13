#ifndef CONFIG_PX4FMU_1_7_H
#define CONFIG_PX4FMU_1_7_H

#define BOARD_PX4FMU

/* PX4 has a 16MHz external clock and 168MHz internal. */
#define EXT_CLK 16000000
#define AHB_CLK 168000000


/*
 * Onboard LEDs
 */
/* red/amber , on PB14 */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_GPIO GPIOB
#define LED_1_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define LED_1_GPIO_PIN GPIO14
#define LED_1_GPIO_ON gpio_clear
#define LED_1_GPIO_OFF gpio_set
#define LED_1_AFIO_REMAP ((void)0)

/* blue, on PB15 */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_GPIO GPIOB
#define LED_2_GPIO_CLK RCC_AHB1ENR_IOPBEN
#define LED_2_GPIO_PIN GPIO15
#define LED_2_GPIO_ON gpio_clear
#define LED_2_GPIO_OFF gpio_set
#define LED_2_AFIO_REMAP ((void)0)



/*
 * UART
 */
#define UART1_GPIO_AF GPIO_AF7
#define UART1_GPIO_PORT_RX GPIOB
#define UART1_GPIO_RX GPIO7
#define UART1_GPIO_PORT_TX GPIOB
#define UART1_GPIO_TX GPIO6

#define UART2_GPIO_AF GPIO_AF7
#define UART2_GPIO_PORT_RX GPIOA
#define UART2_GPIO_RX GPIO3
#define UART2_GPIO_PORT_TX GPIOA
#define UART2_GPIO_TX GPIO2

#define UART6_GPIO_AF GPIO_AF8
#define UART6_GPIO_PORT_RX GPIOC
#define UART6_GPIO_RX GPIO7
#define UART6_GPIO_PORT_TX GPIOC
#define UART6_GPIO_TX GPIO6


/*
 * SPI
 */
/* SPI1 for MPU and accel/gyro if populated */
#define SPI1_GPIO_AF GPIO_AF5
#define SPI1_GPIO_PORT_MISO GPIOA
#define SPI1_GPIO_MISO GPIO6
#define SPI1_GPIO_PORT_MOSI GPIOA
#define SPI1_GPIO_MOSI GPIO7
#define SPI1_GPIO_PORT_SCK GPIOA
#define SPI1_GPIO_SCK GPIO5

/* SPI3 on microSD connector */
#define SPI3_GPIO_AF GPIO_AF5
#define SPI3_GPIO_PORT_MISO GPIOC
#define SPI3_GPIO_MISO GPIO11
#define SPI3_GPIO_PORT_MOSI GPIOB
#define SPI3_GPIO_MOSI GPIO5
#define SPI3_GPIO_PORT_SCK GPIOC
#define SPI3_GPIO_SCK GPIO10

/*
 * SPI slave pin declaration
 */
/* GYRO_CS */
#define SPI_SELECT_SLAVE0_PORT GPIOC
#define SPI_SELECT_SLAVE0_PIN GPIO14

/* ACCEL_CS */
#define SPI_SELECT_SLAVE1_PORT GPIOC
#define SPI_SELECT_SLAVE1_PIN GPIO15

/* MPU_CS */
#define SPI_SELECT_SLAVE2_PORT GPIOB
#define SPI_SELECT_SLAVE2_PIN GPIO0

/* SPI3 NSS on microSD connector */
#define SPI_SELECT_SLAVE3_PORT GPIOA
#define SPI_SELECT_SLAVE3_PIN GPIO4


/* Onboard ADCs */
#define USE_AD_TIM4 1

#define BOARD_ADC_CHANNEL_1 11
#define BOARD_ADC_CHANNEL_2 12
#define BOARD_ADC_CHANNEL_3 13
#define BOARD_ADC_CHANNEL_4 10

#ifndef USE_AD1
#define USE_AD1 1
#endif
/* provide defines that can be used to access the ADC_x in the code or airframe file
 * these directly map to the index number of the 4 adc channels defined above
 * 4th (index 3) is used for bat monitoring by default
 */
#define ADC_1 ADC1_C1
#ifdef USE_ADC_1
#ifndef ADC_1_GPIO_CLOCK_PORT
#define ADC_1_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_1_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1)
#endif
#define USE_AD1_1 1
#else
#define ADC_1_GPIO_CLOCK_PORT 0
#define ADC_1_INIT() {}
#endif

#define ADC_2 ADC1_C2
#ifdef USE_ADC_2
#ifndef ADC_2_GPIO_CLOCK_PORT
#define ADC_2_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_2_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2)
#endif
#define USE_AD1_2 1
#else
#define ADC_2_GPIO_CLOCK_PORT 0
#define ADC_2_INIT() {}
#endif

#define ADC_3 ADC1_C3
#ifdef USE_ADC_3
#ifndef ADC_3_GPIO_CLOCK_PORT
#define ADC_3_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_3_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3)
#endif
#define USE_AD1_3 1
#else
#define ADC_3_GPIO_CLOCK_PORT 0
#define ADC_3_INIT() {}
#endif

#define ADC_4 ADC1_C4
#ifndef ADC_4_GPIO_CLOCK_PORT
#define ADC_4_GPIO_CLOCK_PORT RCC_AHB1ENR_IOPCEN
#define ADC_4_INIT() gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0)
#endif
#define USE_AD1_4 1

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY ADC_4
#endif
#define DefaultVoltageOfAdc(adc) (0.006185*adc)

#define ADC_GPIO_CLOCK_PORT (ADC_1_GPIO_CLOCK_PORT | ADC_2_GPIO_CLOCK_PORT | ADC_3_GPIO_CLOCK_PORT | ADC_4_GPIO_CLOCK_PORT)

/* GPIO mapping for ADC1 pins, overwrites the default in arch/stm32/mcu_periph/adc_arch.c */
#ifdef USE_AD1
#define ADC1_GPIO_INIT(gpio) { \
  ADC_1_INIT(); \
  ADC_2_INIT(); \
  ADC_3_INIT(); \
  ADC_4_INIT(); \
}
#endif // USE_AD1


/*
 * I2C mapping
 */
#define I2C1_GPIO_PORT GPIOB
#define I2C1_GPIO_SCL GPIO8
#define I2C1_GPIO_SDA GPIO9

#define I2C2_GPIO_PORT GPIOB
#define I2C2_GPIO_SCL GPIO10
#define I2C2_GPIO_SDA GPIO11

#define I2C3_GPIO_PORT_SCL GPIOA
#define I2C3_GPIO_SCL GPIO8
#define I2C3_GPIO_PORT_SDA GPIOC
#define I2C3_GPIO_SDA GPIO9


/*
 * PPM
 */
#define USE_PPM_TIM1 1

#define PPM_CHANNEL         TIM_IC1
#define PPM_TIMER_INPUT     TIM_IC_IN_TI1
#define PPM_IRQ             NVIC_TIM1_CC_IRQ
#define PPM_IRQ2            NVIC_TIM1_UP_TIM10_IRQ
// Capture/Compare InteruptEnable and InterruptFlag
#define PPM_CC_IE           TIM_DIER_CC1IE
#define PPM_CC_IF           TIM_SR_CC1IF
#define PPM_GPIO_PORT       GPIOA
#define PPM_GPIO_PIN        GPIO10
#define PPM_GPIO_AF         GPIO_AF1


/*
 * Spektrum
 */
/* The line that is pulled low at power up to initiate the bind process */
#define SPEKTRUM_BIND_PIN GPIO8
#define SPEKTRUM_BIND_PIN_PORT GPIOA

#define SPEKTRUM_UART2_RCC_REG &RCC_APB1ENR
#define SPEKTRUM_UART2_RCC_DEV RCC_APB1ENR_USART2EN
#define SPEKTRUM_UART2_BANK GPIOA
#define SPEKTRUM_UART2_PIN GPIO3
#define SPEKTRUM_UART2_AF GPIO_AF7
#define SPEKTRUM_UART2_IRQ NVIC_USART2_IRQ
#define SPEKTRUM_UART2_ISR usart2_isr
#define SPEKTRUM_UART2_DEV USART2



/* Activate onboard baro */
//#define BOARD_HAS_BARO 1


#endif /* CONFIG_PX4FMU_1_7_H */
