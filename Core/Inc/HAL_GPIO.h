#ifndef HAL_GPIO
#define HAL_GPIO

#include "stm32l4xx.h"

//Port names
#define PORTA GPIOA
#define PORTB GPIOB
#define PORTC GPIOC
#define PORTD GPIOD
#define PORTE GPIOE
#define PORTF GPIOF
#define PORTG GPIOG
#define PORTH GPIOH


//Pin Mode
#define OUTPUT_MODE ((uint32_t) 0x01)
#define INPUT_MODE ((uint32_t) 0x02)

//Input Mode type
#define INPUT_MODE ((uint32_t) 0x00)
#define GENERAL_PURPOSE_OUTPUT_MODE ((uint32_t) 0x01)
#define ALTERNATE_FUNCTION_MODE ((uint32_t) 0x02)
#define ANALOG_MODE ((uint32_t) 0x03)

//output type register
#define OUTPUT_PU_PL ((uint32_t) 0x0)
#define OUTPUT_OPEN_DRAIN ((uint32_t) 0x1)

//output speed
#define LOW_SPEED ((uint32_t) 0x00)
#define MEDIUM_SPEED ((uint32_t) 0x01)
#define HIGH_SPEED ((uint32_t) 0x02)
#define VERY_HIGH_SPEED ((uint32_t) 0x03)

//pull-up/pull-down
#define NO_PU_PD ((uint32_t) 0x00)
#define PU ((uint32_t) 0x01)
#define PD ((uint32_t) 0x02)
#define RESERVED ((uint32_t) 0x03)

//clock enable
#define GPIOA_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<0))
#define GPIOB_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<1))
#define GPIOC_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<2))
#define GPIOD_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<3))
#define GPIOE_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<4))
#define GPIOF_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<5))
#define GPIOG_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<6))
#define GPIOH_CLOCK_ENABLE (RCC->AHB2ENR |= (1<<7))

//Pin position of GPIOx_MODER register
#define MODEx pinNumber
#define MODEy [pinNumber]+1

//configuration structure
typedef struct 
{
  GPIO_TypeDef *port;  
  uint32_t pin;
  uint32_t mode;
  uint32_t input_mode_type;
  uint32_t output_mode_type;
  uint32_t pull;
  uint32_t speed;
}GPIO_TYPE;

//Configure Functions 
//*********************************************
static void config_pin(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinMode);
static void config_output_type(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t outputType);
static void config_pin_speed(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed); 
static void config_pu_pd(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t PuPdType); 

//GPIO user pin function
//****************************************
void gpio_write(GPIO_TypeDef *port, uint32_t pinNumber, uint8_t state);
void gpio_toggle(GPIO_TypeDef *port, uint32_t pinNumber);
void gpio_init(GPIO_TYPE gpio_type);

#endif