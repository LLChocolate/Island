#include "dial_switch.h"
#include "MKL_port.h"

void dial_switch_Init(void)
{
  gpio_init(PTB9,GPI,LOW);
  port_init_NoALT(PTB9,PULLUP);
  gpio_init(PTB10,GPI,LOW);
  port_init_NoALT(PTB10,PULLUP);
  gpio_init(PTB11,GPI,LOW);
  port_init_NoALT(PTB11,PULLUP);
  gpio_init(PTB16,GPI,LOW);
  port_init_NoALT(PTB16,PULLUP);
  gpio_init(PTB17,GPI,LOW);
  port_init_NoALT(PTB17,PULLUP);
  gpio_init(PTB18,GPI,LOW);
  port_init_NoALT(PTB18,PULLUP);
  gpio_init(PTB19,GPI,LOW);
  port_init_NoALT(PTB19,PULLUP);
  gpio_init(PTB20,GPI,LOW);
  port_init_NoALT(PTB20,PULLUP);

}

u16 dial_switch_Scan(void)
{
  u16 status=0,i;
  i=SW1;
  status|=i<<0;
  i=SW2;
  status|=i<<1;
  i=SW3;
  status|=i<<2;
  i=SW4;
  status|=i<<3;
  i=SW5;
  status|=i<<4;
  i=SW6;
  status|=i<<5;
  i=SW7;
  status|=i<<6;
  i=SW8;
  status|=i<<7;
  return status;
}
