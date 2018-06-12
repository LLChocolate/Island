#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"
extern unsigned long int  fangcha_cnt;
extern u8 TimeCnt_Start_Reduct_Flag;
extern void PendSV_Handler(void);         //可悬起的系统调用中断服务函数
extern void HardFault_Handler(void);      //发生硬件上访（程序跑飞）
extern void SysTick_Handler(void);        //滴答时钟(os 用到)
extern void USART1_IRQHandler();          //串口1 中断接收函数
extern void USART2_IRQHandler();
extern void USART3_IRQHandler();
extern void USART5_IRQHandler();
extern void PIT_IRQHandler();            //PIT0 定时中断服务函数
extern void PIT3_IRQHandler();            //PIT0 定时中断服务函数
extern void PIT2_IRQHandler();
extern void PORTA_IRQHandler();           //PORTA中断服务函数
extern void PORTB_IRQHandler();           //PORTB中断服务函数
extern void PORTC_IRQHandler();
//PORTC中断服务函数
extern void PORTE_IRQHandler();
     
extern void FTM0_IRQHandler();            //FTM0输入捕捉中断
extern void FTM1_IRQHandler();            //FTM0输入捕捉中断
extern void VSYNC_IRQ();
extern void DMA0_IRQHandler();
extern void TPM2_IRQHandler();
extern void TPM1_IRQHandler();


#endif
