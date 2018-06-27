/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��KL26 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-12-14
 */

#include "common.h"
#include "include.h"
#include "stdlib.h"

//math common
//signed char L_Diff_temp[70];
//signed char L_Second_Diff_temp[70];
//signed char R_Diff_temp[70];
//signed char R_Second_Diff_temp[70];

u16 Island_Delay_ms = 200;

//Pixel envelope_L[100];
//Pixel envelope_R[100];

u32 L_AD_Ave = 0;
u32 R_AD_Ave = 0;
//u16 TPM2_Cnt = 0;   
//u16 TPM1_Cnt = 0;
Motor_Status Motor1;//���״̬�ṹ��
Motor_Status Motor2;//���״̬�ṹ��
int Speed_L_sum=0;
int Speed_R_sum=0;
s32 Duty_Motor1;//���ռ�ձ�
s32 Duty_Motor2;//���ռ�ձ�
PID_Struct Motor1_PID;//pid�ṹ��
PID_Struct Motor2_PID;//pid�ṹ��
PID_Struct Diff_PID;//����PID
PID_Struct Gyro_PID;//����PID
PID_Struct Diff_Straight;//ֱ��PID
float Speed_goal1=80;//���ת��Ŀ��ֵ
float Speed_goal2=80;//���ת��Ŀ��ֵ
u16 Diff_goal=0;
int Diff_error=0;
u16 Master_Speed=25;
u16 ADC_Value;//ADCֵ
u16 Speed_stand;
int Weight_mean=0;
float stand_half_k;
//***************************��ʱ����־λ*************************
u8 button_timeout=255;//����ʱ���־
u8 key_up = 0;
long int Time_1ms=0;//ʱ����

//****************************************************************
u16 Switch_Status;//����״̬
u8 Key_status;//����״̬
u8 LCD_DISPLAY_FLAG=1;
u8 Motor_enable_Flag=1;
u8 Slow_Flag=0;
u8 Reduct_Flag=0;
u8 Blue_Start_Flag=1;//��������
u8 Key_Start_Flag=0;
//***********************��������ʱȫ�ֱ���**************************
float stand_p;
float stand_d;
float P_TEMP1=500;//201
float I_TEMP1=70;
float D_TEMP1=0;
float P_TEMP2=500;//90
float I_TEMP2=70;
float D_TEMP2=0;
signed int Speed_goal1_TEMP=80;//���ת��Ŀ��ֵ
signed int Speed_goal2_TEMP=80;//���ת��Ŀ��ֵ
u8 Image_Flag=1;
u8 LED_timeout=50;
u8 Dir_temp=1;
u16 hang=0;
int DIFF_UP=15000;
int DIFF_DOWN=-15000;
int Gyro_Up=20000,Gyro_Down=-20000;
long int  temp_cnt=0;
u8 DIFF_PID_CHANGE_FLAG=0;
u16 temp_CNT_WATCH=0;
u8 Speed_max_to_min_diff;
u8 Acc_Limit=40;
u16 stand_AD_L = 0xffff;
u16 stand_AD_R = 0xffff;
u16 stand_AD   = 0Xffff;
float Acc_K=1;

u16 num = 0;
//******************************************************************
u8 image_run_times = 0;
u8 tanzhen = 0;

int _1000ms_cnt = 0;

void main()
{
  u8 Key;
  System_Init();
  Brush_Color=Black;
  Motor1_PID.target=Speed_goal1;
  Motor2_PID.target=Speed_goal2;
  Duty_Motor1=32767;
  Duty_Motor2=32767;
//  while(1)
//  {
//    Speed_goal1 = 30;
//    Speed_goal2 = 30;
////    MOTOR1_DIR=1;
////    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
////    MOTOR2_DIR=1;
////    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
//    SCI_Send_Datas(UART1);
//  }
while(1)
{
//  Motor1_PID.P=P_TEMP1;
//  Motor1_PID.I=I_TEMP1;
//  Motor1_PID.D=D_TEMP1;
//  Motor2_PID.P=P_TEMP2;
//  Motor2_PID.I=I_TEMP2;
//  Motor2_PID.D=D_TEMP2;

//����ͷ�ɼ�һ��
//ͼ���� 

    Key = KEY_Scan();
    if(Key == KEY1_PRES)
    {
      LCD_init(FALSE);
      Disp_single_colour(White);
      LCD_DISPLAY_FLAG = 1;
    }
    else if(Key == KEY2_PRES)
    {
      LCD_DISPLAY_FLAG = 0;
    }
    else if(Key == KEY3_PRES)
    {
      Beep_Once(&Image_Island_Test_Beep);
    }
    if(Image_Flag==1)
    {
      Image_Flag=0;
      ov7725_get_img();//ת�����������������ճ��ж�
      image_process();
      if(LCD_DISPLAY_FLAG==1)
      {
        Send_Image_to_LCD(Image_fire);
        LCD_Draw_Line(Image_lie.Three_Lie[0],Image_lie.Three_lie_end[0],Image_lie.Three_Lie[0],160);  
        LCD_Draw_Line(Image_lie.Three_Lie[1],Image_lie.Three_lie_end[1],Image_lie.Three_Lie[1],160);  
        LCD_Draw_Line(Image_lie.Three_Lie[2],Image_lie.Three_lie_end[2],Image_lie.Three_Lie[2],160);
        LCD_Draw_Line(0,Island.Image_Start_hang,319,Island.Image_Start_hang);
//        LCD_Draw_Line(0,Image_hang.hang_use,319,Image_hang.hang_use);
        LCD_Draw_Line(0,Start_Point,319,Start_Point);
        LCD_Put_Int(100,100,"",L_AD_Ave,Red,White);
        LCD_Put_Int(100,120,"",R_AD_Ave,Red,White);
      }
      image_run_times++;
    }
//  SCI_Send_Datas(UART1);
  if(Motor_enable_Flag==0)
  {
    Speed_stand = 0;
    Speed_goal1=0;
    Speed_goal2=0;
    if(Motor1.Speed<10&&Motor2.Speed<10||Start_line_cnt==2)
    {
      Blue_Start_Flag = 0;
      MOTOR1_DIR=0;
      tpm_pwm_duty(MOTOR_1,0);
      MOTOR2_DIR=0;
      tpm_pwm_duty(MOTOR_2,0);

    }
  }
  
}
}



