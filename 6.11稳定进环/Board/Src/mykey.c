#include "mykey.h"
#include "ISR.h"

void myKEY_Init(void)
{
  gpio_init(PTB23,GPI,HIGH);
  port_init_NoALT(PTB23,PULLUP);
  gpio_init(PTB22,GPI,HIGH);
  port_init_NoALT(PTB22,PULLUP);
  gpio_init(PTC0,GPI,HIGH);
  port_init_NoALT(PTC0,PULLUP);
//  port_init(PTC0,ALT1 | IRQ_FALLING | PULLUP);
//  set_vector_handler(PORTC_PORTD_VECTORn,VSYNC_IRQ);
//  enable_irq (PORTC_PORTD_IRQn);
}

void myKEY_Exti_Init(void)
{
   port_init(PTB1,ALT1 | IRQ_FALLING | PULLUP);
   port_init(PTA17,ALT1 | IRQ_FALLING | PULLUP);
   port_init(PTA15,ALT1 | IRQ_FALLING | PULLUP);
   set_vector_handler(PORTA_VECTORn,PORTA_IRQHandler);
   enable_irq(PORTA_IRQn);
}

u8 KEY_Scan(void)
{
  static u8 key_up=1;//按键按松开标志	  
  if(key_up&&(KEY1==0||KEY2==0||KEY3==0))
  {
//    delayms(10);//去抖动 
    key_up=0;
    if(KEY1==0)                  return KEY1_PRES;
    else if(KEY2==0)            return KEY2_PRES;
    else if(KEY3==0)            return KEY3_PRES;
  }
  else if(KEY1==1&&KEY2==1&&KEY3==1)
  {
    key_up=1; 	    
  }
  return 0;// 无按键按下
//  if(KEY1==0||KEY2==0)//||KEY3==0)
//  {
//    if(key_up == 0)
//   {
//      key_up = 1;
//      button_timeout=15;
//    }
//  }
    
}
