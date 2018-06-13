/*
* 中断处理程序
*/
#include "common.h"
#include "include.h"
#include "define.h"
#include "ISR_FUN.h"
#include "ISR.h"
extern signed int Speed_goal1_TEMP;//电机转速目标值
extern signed int Speed_goal2_TEMP;//电机转速目标值
unsigned long int  fangcha_cnt = 0;
u8 TimeCnt_Start_Reduct_Flag = 1 ;


/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：HardFault_Handler
*  功能说明：硬件上访中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-4    已测试
*  备    注：可以用LED闪烁来指示发生了硬件上访
*************************************************************************/

/*************************************************************************
*  函数名称：VSYNC_IRQ
*  功能说明：PORTD端口中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void VSYNC_IRQ(void)
{
    static u32 flag;
    //Clear Interrupt flag
    flag = PORTC_ISFR;
    PORTC_ISFR = flag;
    
    if(flag & (1<<13))
    {
      Image_Flag=0;
      if(img_flag == IMG_START)	//需要开始采集图像
      {
              //两块存储区循环使用
          //如果之前图像处理使用的存储区是2，则DMA传输的位置是1，则此时把传输位置改为2
          if(Memory_use_Flag==2)
          {
            Memory_use_Flag=1;
            dma_repeat(CAMERA_DMA_CH, (void *)&PTD_B0_IN, (void *)Image_fire_Memory2,CAMERA_SIZE);
          }
          //如果之前图像处理使用的存储区是1，则DMA传输的位置是2，则此时把传输位置改为1
          else if(Memory_use_Flag==1)
          {
            Memory_use_Flag=2;
            dma_repeat(CAMERA_DMA_CH, (void *)&PTD_B0_IN, (void *)Image_fire_Memory1,CAMERA_SIZE);
          }
//        
          DMA_EN(CAMERA_DMA_CH);            		//使能通道CHn 硬件请求
          disable_irq(PORTC_PORTD_IRQn);  
          img_flag=IMG_GATHER;
      }

      else					//图像采集错误
      {
          img_flag = IMG_START;	//开始采集图像
          PORTC_ISFR = flag;		//写1清中断标志位(必须的，不然回导致一开中断就马上触发中断)
          enable_irq(PORTC_PORTD_IRQn); 
      }
    }
    if(flag & (1<<3))
    {
      if(Start_line_flag==0)
      {
        Start_line_flag = 1;
        Start_line_delay = 10000;
        Start_line_cnt++;
        
      }
    }
}

/*************************************************************************
*  函数名称：DMA0_IRQHandler
*  功能说明：DMA0
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要根据自己初始化来修改
*************************************************************************/
void DMA0_IRQHandler()
{
    if(Memory_use_Flag==1)
    {
      Image_fire=&Image_fire_Memory2[0];
    }
    else if(Memory_use_Flag==2)
    {
      Image_fire=&Image_fire_Memory1[0];
    }
    DMA_DIS(CAMERA_DMA_CH);            	//关闭通道CHn 硬件请求
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //清除通道传输中断标志位
    img_flag = IMG_FINISH ; 
    Image_Flag=1;
}

void PendSV_Handler(void)
{
  
}
void HardFault_Handler(void)
{
  MOTOR1_DIR=0;
  tpm_pwm_duty(MOTOR_1,0);
  MOTOR2_DIR=0;
  tpm_pwm_duty(MOTOR_2,0);
}
void SysTick_Handler(void)
{
  
}
void USART5_IRQHandler()
{
  
}

void USART2_IRQHandler()
{

  
}
  

void USART3_IRQHandler()
{

}
void USART1_IRQHandler()
{
  static u8 b_cnt=0;
  static u8 d_cnt = 0;
  const u8 i=0;
  
  u8 j;
  int Sum=0;
  char c;
  char Res_Temp[10]={0};
  static char Res[5];
  DisableInterrupts;
  if(UART_S1_REG(UARTN[UART1]) & UART_S1_RDRF_MASK)
  {
  uart_querystr(UART1,Res_Temp,1);
//  uart_sendStr(UART5,Res_Temp);
  c=Res_Temp[0];
  switch(c)
  {
//  case 'n':
//    {
//      for(j=0;j<i;j++)
//      {
//        Res[j]=0;
//      }
//      i=0;              //清空数组
//      break;
//    }
  case 'm':
    {
//     Debug_Mode=Motor;//调电机PID
      LED2=!LED2;
      break;
    }
  case 's':
    {
//      Debug_Mode=Diff;//调差速PD
      break;
    }
  case 'p':
    {
      for(j=0;j<i;j++)//调P
      {
        Sum+=Res[j]*power_s16(10,i-j-1);
      }
//      if(Debug_Mode==Motor)
//      {
//        Motor1_PID.P=Sum;
//        Motor2_PID.P=Sum;
//      }
//      else if(Debug_Mode==Diff)
//      {
//        Diff_PID.P=Sum;
//      }
      break;
    }
  case 'i':
    {
      for(j=0;j<i;j++)//调I
      {
        Sum+=Res[j]*power_s16(10,i-j-1);
      }
//      if(Debug_Mode==Motor)
//      {
//        Motor1_PID.I=Sum;
//        Motor2_PID.I=Sum;
//      }
//      else if(Debug_Mode==Diff)
//      {
//        Diff_PID.I=Sum;
//      }
      break;
    }
  case 'd':
    {
      if(d_cnt == 0)
      {
        Gyro_PID.target = 30;
        d_cnt++;
      }
      else if(d_cnt == 1)
      {
        Gyro_PID.target = -30;
        d_cnt--;
      }
      break;
    }
  case 'b':
    {
      if(b_cnt==0)
      {
        Blue_Start_Flag=1;
//        Speed_Start_Flag=1;
        b_cnt++;
      }
      else
      {
        Speed_goal1=0;
        Speed_goal2=0;
        Speed_stand=0;
        stand_p=0;
        stand_d=0;
        Motor_enable_Flag=0;
      }
      break;
    }
  case 'a'://停车
    {
      Speed_goal1_TEMP=Speed_goal1;
      Speed_goal2_TEMP=Speed_goal2;
      Speed_goal1=0;
      Speed_goal2=0;
      Speed_stand=0;
      stand_p=0;
      stand_d=0;
      Motor_enable_Flag=0;
      
//      disable()
      break;
//      tpm_pwm_duty(MOTOR_1,0);
//      tpm_pwm_duty(MOTOR_2,0);
    }
  case '1':
    {
      Speed_goal1+=100;
      break;
    }
  case '2':
    {
      Speed_goal1-=100;
      break;
    }
  case '3':
    {
      Speed_goal2+=100;
      break;
    }
  case '4':
    {
      Speed_goal2-=100;
      break;
    }
    
  default:
  {
    if(c>=48&&c<=57)//如果是数字，则有效
    {
//      i=strlen(Res_Temp);
      for(j=0;j<i;j++)
      {
        Res[j]=Res_Temp[j]-48;
      }
//      printf("\r\n%d\r\n",i);
    }
    
    break;
  }
  }
//  if(Debug_Mode==Motor)
//  {
//    printf("\r\nMode:Motor\r\n");
//  }
//  else if(Debug_Mode==Diff)
//  {
//    printf("\r\nMode:Diff\r\n");
//  }
  printf("\r\nRes:");
  
  for(j=0;j<i;j++)
  {
    uart_putchar(UART1,Res[j]+48);
  }
  printf("\r\n");

  }
  EnableInterrupts;
}


void PIT_IRQHandler()
{
  
  static unsigned char TimeCnt_20ms = 0,TimeCnt_5ms=0;	  //5ms,20ms时间计数器    
  static unsigned int  TimeCnt_2000ms = 0;
  static unsigned int  TimeCnt_Key_Start_ms = 0;
  static unsigned int  TimeCnt_Start_Reduct_ms = 0;

  //时间标尺更新 
  Time_1ms++; 
  TimeCnt_5ms++;
  TimeCnt_20ms++;
  if(Reduct_Flag==1&&Blue_Start_Flag==1)
    TimeCnt_2000ms++;
  if(Key_Start_Flag==1)
    TimeCnt_Key_Start_ms++;
  if(Blue_Start_Flag==1&&TimeCnt_Start_Reduct_ms<1002)
  {
    TimeCnt_Start_Reduct_ms++;
    if(TimeCnt_Start_Reduct_ms==1000)
    TimeCnt_Start_Reduct_Flag=0;
  }
  if(TimeCnt_5ms >= 5)
    TimeCnt_5ms = 0;
  if(TimeCnt_20ms >= 20)
    TimeCnt_20ms = 0;
//*************************环岛时间标志***********************
  if(Island.Next_Island_flag==1)
  {
    if(Island.Next_Island_flag_delay==0)
    {
      Island.State = NoIsland;//清标志
      Island.In_Center = 0;
      Island.In2Stay_cnt = 0;
      Island.Out_Center = 0;
      Island.Out_Allow_flag = 0;
      Island.Stay2Out_cnt = 0;
      Island.Next_Island_flag = 0;
    }
    Island.Next_Island_flag_delay--;
  }
  if(Island.Stay2Out_flag==1)
  {
    if(Island.Stay2Out_flag_delay==0)
    {
      Island.Out_Allow_flag = 1;
      Island.Stay2Out_flag = 0;
    }
    Island.Stay2Out_flag_delay--;
  }
  
//*****************************END****************************
  
  
//***********************Beep时间标志*******************
  Beep_ISR(&Image_Island_Test_Beep);
//***************************END******************
  if(TimeCnt_Key_Start_ms >= 3000)
  {
    Blue_Start_Flag=1;
    Key_Start_Flag=0;
  }

//PIT0服务函数
//按键消抖
    if(button_timeout<16)
    {
      button_timeout--;
    }
  if(button_timeout==0)//延时10ms检测按键状态
  {
//    key_up = 0;
      if(KEY1==0)
      {
      }
      else if(KEY2==0)
      {
      }
      else if(KEY3==0)
      {
        Key_status=KEY3_PRES;
        Blue_Start_Flag = 1;
        LED3 = !LED3;
      }
  }

//按键消抖函数结束
  
  
//***************************************************差速函数
  if(TimeCnt_5ms==1)
  {
  }
  
//****************************************************测速函数
  if(TimeCnt_5ms==2&&Blue_Start_Flag==1)//20ms执行一次
  {
    Get_speed1(&Motor1);
    Get_speed2(&Motor2);
    Speed_Control();
    Speed_output();
    L_AD_Ave = adc_once(MYADC_2,ADC_12bit);
    R_AD_Ave = adc_once(MYADC_3,ADC_12bit);
//    AD_new();
    
  }
    //*****************************************************adc检测

  if(Time_1ms == 1000)
  {
    stand_AD_L = L_AD_Ave;
    stand_AD_R = R_AD_Ave;
    if(stand_AD_L>10000||stand_AD_R>10000)
    {
    }
    stand_AD = (stand_AD_L+stand_AD_R)/2;
  }
  
  //***************************MPU6050*****************


  PIT_Flag_Clear(PIT0);
}



void PORTC_IRQHandler()
{

}
void PORTA_IRQHandler()
{
  u8 n=0;
  n=17;
  if(PORTA_ISFR & (1<<n))
  {
//按键中断1的中断服务
    button_timeout=15;//标志置15
    PORTA_ISFR |=(1<<n);//写入1清空中断标志位
  }
  n=15;
  if(PORTA_ISFR & (1<<n))
  {
//按键中断1的中断服务
    button_timeout=15;//标志置15
    PORTA_ISFR |=(1<<n);//写入1清空中断标志位
  }
}

void PORTB_IRQHandler()
{
  u8 n=0;
  n=1;
  if(PORTB_ISFR & (1<<n))
  {
//按键中断1的中断服务
    button_timeout=15;//标志置15
    PORTB_ISFR |=(1<<n);//写入1清空中断标志位
  }
}

void PORTE_IRQHandler()
{

}
void FTM0_IRQHandler()
{
  
}
void FTM1_IRQHandler()
{
  
}

//void TPM2_IRQHandler()
//{
////  u8 s;
////  s = TPM2_STATUS;
//  TPM2_STATUS = ~0;
//  TPM2_STATUS = ~0;
////  if(s & (1 << 0))
////  {
//    TPM2_Cnt++;
////  }
//}


//void TPM1_IRQHandler()
//{
////  u8 s;
////  s = TPM1_STATUS;
//  TPM1_STATUS = ~0;
//  TPM1_STATUS = ~0;
////  if(s & (1 << 0))
////  {
//    TPM1_Cnt++;
////  }
//}



