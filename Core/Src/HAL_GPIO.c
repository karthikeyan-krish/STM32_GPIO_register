#include "HAL_GPIO.h"
#include <stdint.h>

//To calculate register position based on pin number
uint32_t PINPOS[16] = {
    (0x00),
    (0x02),
    (0x04),
    (0x06),
    (0x08),
    (0x0A),
    (0x0C),
    (0x0E),
    (0x10),
    (0x12),
    (0x14),
    (0x16),
    (0x18),
    (0x1A),
    (0x1C),
    (0x1E)
};

static void config_pin(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinMode)
{
    /*switch(pinMode)
    {
        case INPUT_MODE:
        port->MODER &= ~((1<<MODEx) | (1<<MODEy));
        break;

        case GENERAL_PURPOSE_OUTPUT_MODE:
        port->MODER &= ~(1<<MODEy); 
        port->MODER |= (1<<MODEx);
        break;

        case ALTERNATE_FUNCTION_MODE:
        port->MODER |= (1<<MODEy);
        break;

        case ANALOG_MODE:
        port->MODER |= ((1<<MODEx) | (1<<MODEy));
        break;

    }*/
    port->MODER &= ~((1<<PINPOS[pinNumber]) | (1<<(PINPOS[pinNumber]+1))); // By default the GPIOx_MODER is set to Analog mode (11), so set register value 0x00 before seting the mode.
    port->MODER |= (pinMode<<(PINPOS[pinNumber]));


}


static void config_pin_speed(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t pinSpeed)
{
    port->OSPEEDR |= (pinSpeed<< (PINPOS[pinNumber]));
}

static void config_output_type(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t outputType)
{
    port->OTYPER |= (outputType<< (pinNumber));
}

static void config_pu_pd(GPIO_TypeDef *port, uint32_t pinNumber, uint32_t PuPdType)
{
    port->PUPDR |= (PuPdType<< (PINPOS[pinNumber]));
}

void gpio_write(GPIO_TypeDef *port, uint32_t pinNumber, uint8_t state)
{
    if(state)
    {
        port->BSRR = (1<<pinNumber);

    }
    else
    {
        port->BSRR = (1<<(pinNumber + 16));
    }
}

void gpio_toggle(GPIO_TypeDef *port, uint32_t pinNumber)
{
    port->ODR ^= (1<<pinNumber);
}


void gpio_init(GPIO_TYPE gpio_type)
{
    
    if(gpio_type.port == PORTA)
    {
        GPIOA_CLOCK_ENABLE;
        
    }

    if(gpio_type.port == PORTB)
    {
        GPIOB_CLOCK_ENABLE;
        
    }
    
    if(gpio_type.port == PORTC)
    {
        GPIOC_CLOCK_ENABLE;
        
    }

    if(gpio_type.port == PORTD)
    {
        GPIOD_CLOCK_ENABLE; 
        
    } 

    config_pin(gpio_type.port, gpio_type.pin, gpio_type.input_mode_type);
    config_output_type(gpio_type.port, gpio_type.pin, gpio_type.output_mode_type);
    config_pin_speed(gpio_type.port, gpio_type.pin, gpio_type.speed); 
    config_pu_pd(gpio_type.port, gpio_type.pin, gpio_type.pull); 
}