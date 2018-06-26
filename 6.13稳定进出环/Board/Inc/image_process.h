#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "common.h"
//寻找黑线并寻找中心值
#define CENTER_             160
#define HALF_WIDTH          98
#define HALF_NEAR           115
#define VN_HALF             170
#define Island_HALF         106
#define Far_Point           85            //1m处  76半宽
#define Far_Half            80
#define Start_Point         105         //45cm处
#define Near_Point          150            //近处
#define VN_Point            180
#define Island_Point        125

#define Image_Point(x,y)    (Image_fire[(x)][(y)/8]>>(7-((y)%8))&1)
#define AD_Near_hang        120
#define Turn_Point          100

#define Center_correct(hang)   ((hang-Start_Point)*(4-4)*1.0/(Far_Point-Start_Point)+4)


#define _1m_Left        108
#define _1m_Right       198
#define Near_Left       76
#define Near_Right      230


#define  Centerdiff_const    10
#define  Center_Filter_Period 5

#define Island_Center_Period_Const  (10)
#define Cross_Center_Period_Const  (10)

typedef enum
{
  turn_left  = 1,
  turn_right = 2,
  go_str     = 3
}_1m_Road_Status;
void image_process(void);
u8 get_black_line(unsigned char *ImageData_in,int hang);//捕捉黑线 
u8 CenterlineToDiff(int center);
void get_three_lie(void);
u8 Cross_Test(void);
u8 double_AD(void);
u8 Island_process(void);
u8 Elec_Island(void);
u8 In_Island(void);
int In_Island_center(u8* hang);
u8 Stay_Island(void);
u8 Out_Island(void);
u8 Image_Island_Test(void);
int Test_Far_Lie(void);
int Out_Island_Test(int* start_end, int* end_end);
int Stay2Out_test();
int Wait_Next_Island();
int Wait_Next_center(u8* hang);
u8 Cross_process(void);
u8 In_Cross_test();//斜入十字检测
u8 In_Cross();//斜入十字
u8 Cross_center_test(int* start_end, int* end_end);//和出环岛时找中心点的代码一样
u8 Cross_curve_test();
u8 Out_Cross_test(void);
u8 Cross_pre_test(void);
#endif