#include "init.h"

void System_Init()
{
  DisableInterrupts;
  myBEEP_Init();
  
  pit_init_ms(PIT0,1);
  set_vector_handler(PIT_VECTORn,PIT_IRQHandler);
  enable_irq(PIT_IRQn);
  
  dial_switch_Init();
  Parameters_init_CAR();
//  MY_adc_init();
  MYADC_INIT();
  getspeed1_init();
  getspeed2_init();
  myLED_Init();
  Motot1_Init();
  Motot2_Init();
//  FTM_PWM_init(FTM0,CH2,10000,30000);
  myKEY_Init();
  MPU_Init(98);
//  myKEY_Init();
  if(LCD_DISPLAY_FLAG==1)
  {
    LCD_init(FALSE);
    Disp_single_colour(White);
  }

  PID_Init(&Motor1_PID,800,60,0,50000,50,50000,-50000);//600
  PID_Init(&Motor2_PID,800,60,0,50000,50,50000,-50000);//600
//  PID_Init(&Diff_PID,0.85,0,0.25,0,0,DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
  //                                                  60:0.32                          limit:35
  //                                                  70:0.4 limit:50
  //                                                  80:0.6  0.04  limit:80
  //                                                  90:1  0.07  limit:85
  //                                                    90:0.3
  //                                                    110:0.5 0.1  90
  //                                                    120:0.7 0.14  100
  //                                                    130:0.6 0.15    110
  //                                                    140:1.25 0.25 118
  //                                                    145:0.8 0.23
  //                                                    150:0.85 0.25
  //                                                    153:0.96 0.25
  //                                                    155:1   0.28  110
  stand_p=Diff_PID.P;
  stand_d=Diff_PID.D;
//  PID_Init(&Diff_Straight,0.04,0,0,0,0,DIFF_UP,DIFF_DOWN);
//  stand_p=
  
  set_vector_handler(UART1_VECTORn,USART1_IRQHandler);
  uart_rx_irq_en(UART1);
  
//  Brush_Color=Red;			//：|：：???│?：???ao：?：|? 
//  Back_Color=White;
  Ov7725_Init();
  EnableInterrupts;
  return ;
}


void Parameters_init_CAR(void)
{
  u8 i;
  stand_half_k=(HALF_WIDTH-Far_Half)*1.0/(Start_Point-Far_Point);
  for(i=Far_Point;i<200;i++)
  {
    Image_hang.center[i] = 160;
    Image_hang.halfwidth[i] = (i-Far_Point)*(80-24)*1.0/(Start_Point-Far_Point)+24;
  }
  Switch_Status=dial_switch_Scan();
  if((Switch_Status&3)==0)
  {
    Speed_stand=70;
    Speed_goal1 = Speed_stand;
    PID_Init(&Diff_PID,0.13,0,0.05,0,0,DIFF_UP,DIFF_DOWN);//DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
    PID_Init(&Gyro_PID,150,0,1,0,0,Gyro_Up,Gyro_Down);
  }
  else if((Switch_Status&3)==1)
  {
    Speed_stand=150;
    PID_Init(&Diff_PID,0.85,0,0.25,0,0,DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
  }
  else if((Switch_Status&3)==2)
  {
    Speed_stand=155;
    PID_Init(&Diff_PID,0.95,0,0.3,0,0,DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
  }
  else if((Switch_Status&3)==3)
  {
    Speed_stand=160;
    PID_Init(&Diff_PID,1.08,0,0.36,0,0,DIFF_UP,DIFF_DOWN);//30:0.08,40:0.09,50:0.13  limit:20
  }
  
//  if(((Switch_Status>>2)&3)==0)
//  {
//    Acc_K=1;
//  }
//  else if(((Switch_Status>>2)&3)==1)
//  {
//    Acc_K=1.15;
//  }
//  else if(((Switch_Status>>2)&3)==2)
//  {
//    Acc_K=1.3;
//  }
//  else if(((Switch_Status>>2)&3)==3)
//  {
//    Acc_K=1.45;
//  }
  
//  if(((Switch_Status>>4)&3)==0)
//  {
//    Acc_Limit=30;
//  }
//  else if(((Switch_Status>>4)&3)==1)
//  {
//    Acc_Limit=50;
//  }
//  else if(((Switch_Status>>4)&3)==2)
//  {
//    Acc_Limit=60;
//  }
//  else if(((Switch_Status>>4)&3)==3)
//  {
//    Acc_Limit=70;
//  }
  
  if(((Switch_Status>>4)&1)==1)
  {
    LCD_DISPLAY_FLAG=1;
  }
  else 
  {
    LCD_DISPLAY_FLAG=0;
  }
//  else if(Switch_Status==2)
//  {
//
//  }
}