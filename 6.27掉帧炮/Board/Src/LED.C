#include "led.h"

void myLED_Init(void)
{
  gpio_init(PTC5,GPO,HIGH);
  gpio_init(PTC6,GPO,HIGH);
  gpio_init(PTC7,GPO,HIGH);
  gpio_init(PTC8,GPO,HIGH);
}


void LED_Open_once(void)
{
  LED1=1;
  delayms(10);
  LED1=0;
}
