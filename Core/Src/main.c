#include "stm32l4xx.h"
#include "HAL_GPIO.h"
int main()
{
  HAL_Init();

  //****************************************************************
  /*RCC->AHB2ENR |= (1<<1); // Enabling AHB2 RCC clock for GPIOC pin
  //GPIOB->MODER |= (0x01<<(28));
  GPIOB->MODER &= ~(1<<29); // setting pin 14 in GPO mode (LED2)
  GPIOB->MODER |= (1<<28);
  GPIOB->OTYPER &= ~(1<<14); // setting pin 14 in push-pull
  GPIOB->OSPEEDR |= (1<<29);
  //GPIOB->OSPEEDR &= ~(1<<28);
  GPIOB->PUPDR &= ~((1<<28) | (1<<29));*/
  //****************************************************************

  GPIO_TYPE myGPIO;

  myGPIO.port = PORTB;
  myGPIO.pin = 14;
  myGPIO.mode = OUTPUT_MODE;
  myGPIO.input_mode_type = GENERAL_PURPOSE_OUTPUT_MODE;
  myGPIO.output_mode_type = OUTPUT_PU_PL;
  myGPIO.pull = NO_PU_PD;
  myGPIO.speed = HIGH_SPEED;

  gpio_init(myGPIO);

  while(1)
  {
  

  //***************************************************************
  /*GPIOB->BSRR = (1<<14); //set pin 14 high
  HAL_Delay(1000);
  GPIOB->BSRR = (1<<30); //set the pin 14 low
  HAL_Delay(1000);*/
  //*************************************************************
  
  gpio_toggle(PORTB,14);
  HAL_Delay(1000);

  }
}