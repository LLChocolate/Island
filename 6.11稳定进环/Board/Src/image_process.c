////环岛判断：1.前方直道
////          2.半宽异常（判断环岛左右后锁定）
////          3.判断不为直道后判断进入环岛时前方碰到的角度的形状
////          4.出环岛时判断前方竖直三线（三线尽头的位置连续跳变三次）
////          5.跳变结束后DElay 200ms后解除环岛锁定
//
//
//
#include "image_process.h"
#include "include.h"
#include "stdlib.h"
u8 ImageData[320];
Edge_Str Island_mark_edge[50];
float island_addline_k = 0;
float Cur_error = 0;

u8 diff_done_flag = 0;
u8 Island_out_flag = 0;
u16 Island_in_delay = 300;
u16 Island_out_delay = 300;

u8 Island_In_Flag = 0;

u16 Start_line_delay = 0xffff;
u8  Start_line_flag  = 0;
u8  Start_line_cnt   = 0;

u8  road_filter_flag = 0;
u8  Far_correct_flag = 0;
u8  Far_Diff = 0;
u8  Island_Center_Lock_flag = 0;
u8  Angel_Find_Flag = 0;
Filter_1st_Str Center_Filter = {0.5,{0,0},{0,0}};
//Kalman_Date Center_Filter={0,0,0,0,0,0.1,40,0,0};
s16 fangcha_test[200]={0};

u8 Cross_flag;
u16 Cross_flag_delay;
u16 Cross_flag_delay_const = 1000;


Image_hangData Image_hang;
Image_lieData  Image_lie={{78,158,238},{0}};
Island_Data    Island={
                  .Correct_hang = 150,
                  .Image_Start_hang = 83,
                  .Next_Island_flag_delay_const = 1000,
                  .Stay2Out_flag_delay_const = 800
                    };
void image_process(void)
{
//  u8 max_temp;
//  u8 min_temp;
  diff_done_flag = 0;//每次循环的开头将diff传值标志置零
  Image_hang.hang_use = 0;
  get_three_lie();
  if(Image_lie.Three_lie_end[1]<65)
    road_filter_flag = 1;
  else
    road_filter_flag = 0;
//  get_black_line(Image_fire[Far_Point],Far_Point);//45cm处中心点
//  get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
  if(Cross_Test()==1)
  {
    Road_Status_Flag = Cross;
  }
  Island_process();
//  Out_Island();
    Slow_Flag=0;
    if(Island.State!=NoIsland&&Island.State!=Wait_Next)
    {
      
    }
    else if(Road_Status_Flag==Cross)
    {
      get_black_line(Image_fire[Far_Point],Far_Point);//75cm处中心点
      CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
    }
    else
    {
      Slow_Flag=1;
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
    }
    if(LCD_DISPLAY_FLAG==1)
    {
      LCD_Put_Int(250,100,"cen:",Image_hang.center[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,120,"half",Image_hang.halfwidth[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,140,"L",Image_hang.getLeft_flag[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,160,"L_V",Image_hang.black_L[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,180,"R",Image_hang.getRight_flag[Image_hang.hang_use],Red,White);
      LCD_Put_Int(250,200,"R_V",Image_hang.black_R[Image_hang.hang_use],Red,White);
      switch(Island.State)
      {
      case NoIsland:
        LCD_PutString(250,220,"N",Red,White);
        break;
      case Left_Island_pre:
        LCD_PutString(250,220,"Lp",Red,White);
        break;
      case Right_Island_pre:
        LCD_PutString(250,220,"Rp",Red,White);
        break;
      case Left_Island_in:
        LCD_PutString(250,220,"Li",Red,White);
        break;
      case Right_Island_in:
        LCD_PutString(250,220,"Ri",Red,White);
        break;
      case Left_Island_out:
        LCD_PutString(250,220,"Lo",Red,White);
        break;
      case Right_Island_out:
        LCD_PutString(250,220,"Ro",Red,White);
        break;
      case Wait_Next:
        LCD_PutString(250,220,"W",Red,White);
        break;
      }
    }
    DIFF_PID_CHANGE_FLAG=0;//使用直道PID
    Road_Status_Flag=Straight;

}

//
u8 get_black_line(unsigned char *ImageData_in,int hang)//捕捉黑线  
{
  int Middle=160;  //黑线中间默认为CENNTER
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int getleft_flag=0,getright_flag=0;//找到左右标志0
  int _black_R,_black_L;//黑线左右端
  int _halfwidth = 100;//黑线一半宽度默认80
  static unsigned char first_run=0;//开跑点0
  u8 middle_black_flag = 0;
  int i=0;
  if(first_run==0)  //开跑点是0处
  {
    
    first_run++;//开跑点加1
  }
  else//如果first_run!=0
  {
    Middle = (Image_hang.center[hang]+Image_lie.Three_Lie[1])/2;
    _halfwidth = Image_hang.halfwidth[hang];//halfwidth[hang];//一半为halfwidth[hang]
  }
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Right_Count = Middle;//把黑线中间值赋给右计数起点
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
    Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
  
  if(Right_Count<ccd_end)//如果在有效范围内
  {
    _black_R = Right_Count;
    Image_hang.getRight_flag[hang]=1;
    getright_flag=1;
  }
  else
  {
    getright_flag=0;
    Image_hang.getRight_flag[hang]=0;
  }
  Left_Count = Middle;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count > ccd_start)
  {
    _black_L = Left_Count; 
    Image_hang.getLeft_flag[hang]=1;
    getleft_flag=1;
  } 
  else
  {
    getleft_flag=0;
    Image_hang.getLeft_flag[hang]=0;
  }
  if(Left_Count==Middle||Right_Count==Middle)//中间部分为黑色
  {
    getright_flag=0;
    Image_hang.getRight_flag[hang]=0;
    getleft_flag=0;
    Image_hang.getLeft_flag[hang]=0;//flag先清零
    middle_black_flag = 1;
  }
  else
  {
    middle_black_flag = 0;
  }
  

  if(middle_black_flag==1&&hang<210)
  {
    Image_hang.center[hang+3] = Middle;
    Image_hang.halfwidth[hang+3]=_halfwidth+8;
    get_black_line(Image_fire[hang+3],hang+3);
  }
  else if(getleft_flag==0 && getright_flag==0)//左右边界都没有找到
  {
    
  }
  else if(getleft_flag!=1 && getright_flag==1)//找到右边界
  {
    Middle = _black_R-_halfwidth;//黑线中间位置为右边界-黑线宽的一半
    _black_L = _black_R - _halfwidth*2;//黑线左边位置为右边界-黑线宽
  }
  else if(getleft_flag==1 && getright_flag!=1)//找到左边界
  {
    Middle = _black_L+_halfwidth;
    _black_R = _black_L + _halfwidth*2;
  }
  else if(getleft_flag==1 && getright_flag==1) //左右边界都找到
  {
      _halfwidth=(int)((_black_R - _black_L)/2.0); //如果检测到的左右差值超出160，取中间位置
    if(_halfwidth < 100)//宽度限幅 
      _halfwidth = 100;
    else if(_halfwidth >140)
      _halfwidth = 140; 
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  if(Middle<35) //中心点限幅 
    Middle=35;
  else if(Middle>285)
    Middle=285;
  
  //data record 记录参数到数组中
  Image_hang.center[hang] = Middle + Center_correct(hang);
  if(Image_hang.hang_use<hang)
  {
    Image_hang.hang_use = hang;
  }
  if(_black_L>319)_black_L=319;
  else if(_black_L<0)_black_L=0;
  Image_hang.black_L[hang] = _black_L;
  if(_black_R>319)_black_R=319;
  else if(_black_R<0)_black_R=0;
  Image_hang.black_R[hang] = _black_R;
  Image_hang.halfwidth[hang] = _halfwidth;
  return 0;
}


void get_three_lie(void)
{
  u16 left_lie=Image_lie.Three_Lie[0],middle_lie=Image_lie.Three_Lie[1],right_lie=Image_lie.Three_Lie[2];
  u8 Left_flag=0,Right_flag=0,middle_flag=0;
  u8 Left_point,Middle_point,Right_point;
  Left_point=239;
  while(!(Image_Point(Left_point,left_lie)==1
          &&Image_Point(Left_point-1,left_lie)==1
            &&Image_Point(Left_point-2,left_lie)==1)&&Left_point>=2)
    Left_point--;
  if(Left_point!=239)//左线初始不为黑
  {
    Left_flag=1;
  }
  Image_lie.Three_lie_end[0]=Left_point;
  Middle_point=239;
  while(!(Image_Point(Middle_point,middle_lie)==1
          &&Image_Point(Middle_point-1,middle_lie)==1
            &&Image_Point(Middle_point-2,middle_lie)==1)&&Middle_point>=2)
    Middle_point--;
  if(Middle_point!=239)//左线初始不为黑
  {
    middle_flag=1;
  }
  
  Image_lie.Three_lie_end[1]=Middle_point;
  Right_point=239;
  while(!(Image_Point(Right_point,right_lie)==1
          &&Image_Point(Right_point-1,right_lie)==1
            &&Image_Point(Right_point-2,right_lie)==1)&&Right_point>=2)
    Right_point--;
  if(Right_point!=239)//左线初始不为黑
  {
    Right_flag=1;
  }
  Image_lie.Three_lie_end[2]=Right_point;
  if(Left_flag==0)
    Road_Status_Flag=Right_turn;
  else if(Right_flag==0)
    Road_Status_Flag=Left_turn;
  else 
  {
    if(Left_point<Middle_point&&Middle_point<Right_point)
    {
      Road_Status_Flag=Left_turn;
    }
    else if(Left_point>Middle_point&&Middle_point>Right_point)
    {
      Road_Status_Flag=Right_turn;
    }
  }
}




u8 CenterlineToDiff(int center)
{
  static u8  center_period = 0;
  static u16 center_old[Center_Filter_Period];
  static u16 center_ave = 0;
  if(diff_done_flag == 1)return 0;
  

//*********************对中心点均值滤波*************************************  

//  if(center_period > (Center_Filter_Period - 1))
//  center_period = 0;
//  
//  center_ave -= center_old[center_period];
//  center_old[center_period] = center;
//  center_ave += center_old[center_period];
//  center_period++;  
  
  diff_done_flag = 1;
  Diff_error = 160 - center;
  
  return 0;
}


u8 Cross_Test(void)
{
  u8 i,cnt=0;
  for(i=0;i<5;i++)
  {
    if(sum_point(Image_fire[Start_Point+i],40)<=5)
      cnt++;
  }
  if(cnt>3)
  {//BEEP=1;
  return 1;}
  else 
  {//BEEP=0;
  return 0;}
}

u8 double_AD(void)
{
//  if((L_AD_Ave>stand_AD_L*1.8)&&(R_AD_Ave>stand_AD_R*1.8))
  if((L_AD_Ave>3000)||(R_AD_Ave>3000))
  {
//    BEEP = 1;
    return 1;
  }
  else 
  {
//    BEEP = 0;
    return 0;
  }
}

u8 Island_process(void)
{
  Elec_Island();//电磁检测，只在入环和出环的时候检测
  In_Island();//入环岛
  Stay_Island();//在环岛里
  Out_Island();//出环岛
  return 0;
}

u8 Elec_Island(void)
{
  u8 doublt_island = 0;
  if(Island.State == NoIsland)//无环岛时检测环岛
  {
    doublt_island = Image_Island_Test();
    if(doublt_island!=0&&double_AD()==1)
    {
      Island.State = doublt_island;
      LED1 = 0;
    }
  }
  
  if(Island.State == Left_Island_out
     ||Island.State == Right_Island_out)
  {
    if(double_AD()==1)
    {
      Island.State = Wait_Next;//等待下一个环岛的时间间隔
      Island.Next_Island_flag = 1;
      Island.Next_Island_flag_delay = Island.Next_Island_flag_delay_const;
    }
      
  }
  return 0;
}

u8 In_Island(void)
{
  int center;
  int center_use;
  u8  Impulse_hang = 0;
  if(Island.State!=Left_Island_pre&&Island.State!=Right_Island_pre)//不在此状态下
    return 1;//直接返回
  else 
  {
    center = In_Island_center(&Impulse_hang);//寻找中心点
    if(center == -1)//寻找失败或者已完全进入环岛
    {
      Island.In2Stay_cnt ++;
      CenterlineToDiff(Island.In_Center);//使用上一次的旧值
      if(Island.In2Stay_cnt>=10)//连续10次寻找失败，认为进入环岛，更新状态
      {
        if(Island.State==Left_Island_pre)
        {
          Island.State = Left_Island_in;
          Island.Stay2Out_flag = 1;
          Island.Stay2Out_flag_delay = Island.Stay2Out_flag_delay_const;
        }
        else if(Island.State==Right_Island_pre)
        {
          Island.State = Right_Island_in;
          Island.Stay2Out_flag = 1;
          Island.Stay2Out_flag_delay = Island.Stay2Out_flag_delay_const;
        }
      }
    }
    else
    {
      if(Island.State==Left_Island_pre)
      {
        if(center>Image_lie.Three_Lie[1]+30)//突变点太靠右
        {
          Island.In2Stay_cnt++;
          get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
          if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
             &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
               &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
          {
            get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
          }
          center_use = Image_hang.center[Image_hang.hang_use];
        }
        else
        {
          Island.In2Stay_cnt = 0;//清零
          center_use = ((center - (center - 319)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 0)/2 + 10;//布线（三角形相似）
        }
        Island.In_Center = center_use;//保存上一次的中心点
        CenterlineToDiff(center_use);
      }
      else if(Island.State==Right_Island_pre)
      {
        if(center<Image_lie.Three_Lie[1]-30)//突变点太靠左
        {
          Island.In2Stay_cnt++;
          get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
          if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
             &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
               &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
          {
            get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
          }
          center_use = Image_hang.center[Image_hang.hang_use];
        }
        else
        {
          Island.In2Stay_cnt = 0;//清零
          center_use = ((center - (center - 0)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 319)/2 - 10;
        }
        Island.In_Center = center_use;//保存上一次的中心点
        CenterlineToDiff(center_use);
      }
      //调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Blue);//行列颠倒
      }
    }
  }
  return 0;
}

int In_Island_center(u8* hang)//入环岛时寻找突变点+补线
{
  int Middle;
  int center;
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int Diff_L[19],Diff_R[19];//一阶差分
  int DDiff_L[18],DDiff_R[18];//二阶差分
  int Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u16 In_black_L[20];
  u16 In_black_R[20];
  u8  Impulse_L_Flag = 0,Impulse_R_Flag = 0;//一阶差分中出现阶跃
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  Middle = Test_Far_Lie();//从65行开始到255行
  if(Island.State == Right_Island_pre)
  {
    if(Middle>200)return -1;//前方直道已经看不到了
  }
  else if(Island.State == Left_Island_pre)
  {
    if(Middle<120)return -1;//前方直道已经看不到了
  }
  
  
  for(i=0;i<20;i++)//20行
  {
    ImageData_in = Image_fire[i*3+Island.Image_Start_hang];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    if(Island.State == Right_Island_pre)
    {
      Right_Count = Middle;//把黑线中间值赋给右计数起点
      while(!(ImageData[Right_Count+3]==1 
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
      if(Right_Count<ccd_end)//如果在有效范围内
      {
        In_black_R[i] = Right_Count;
      }
      else if(Right_Count<Image_lie.Three_Lie[1]+10)
      {
        In_black_R[i] = ccd_end;
      }
      else
      {
        In_black_R[i] = ccd_end;
      }
    }
    else if(Island.State == Left_Island_pre)
    {
      Left_Count = Image_lie.Three_Lie[1];
      while(!(ImageData[Left_Count-3]==1 
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        In_black_L[i] = Left_Count; 
      }
      else if(Left_Count>Image_lie.Three_Lie[1]-10)
      {
        In_black_L[i] = ccd_start;
      }
      else
      {
        In_black_L[i] = ccd_start;
      }
    }
  }
  for(i=0;i<19;i++)
  {
    if(Island.State == Right_Island_pre)
    {
      Diff_R[i] = In_black_R[i+1] - In_black_R[i];
    }
    else if(Island.State == Left_Island_pre)
    {
      Diff_L[i] = In_black_L[i+1] - In_black_L[i];
    }
  }
  for(i=0;i<18;i++)
  {
    if(Island.State == Right_Island_pre)
    {
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_R[i])<3)Liner_R_cnt++;
      if(DDiff_R[i]<-30&&Liner_R_cnt>i-3&&Liner_R_cnt>1)
      {
        Impulse_R_Flag = 1;
        center = In_black_R[i];//出现跳转的行
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
    else if(Island.State == Left_Island_pre)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      if(Abs_(DDiff_L[i])<3)Liner_L_cnt++;
      if(DDiff_L[i]> 30&&Liner_L_cnt>i-3&&Liner_L_cnt>1)
      {
        Impulse_L_Flag = 1;//出现冲激
        center = In_black_L[i];//出现跳转的行
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
  }
  if(Impulse_R_Flag==0&&Impulse_L_Flag==0)
  {
    return -1;//没有出现
  }
  else 
  {
    return center;
  }
}

#define Island_Center_Period_Const  (10)
u8 Stay_Island(void)
{
  if(Island.State!=Left_Island_in&&Island.State!=Right_Island_in)
    return 1;
  
//当作普通弯道寻找中线
  get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
  if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
     &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
       &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
  {
    get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
  }
  CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  
  if(Island.Out_Allow_flag==1)//允许改变状态
  {
    if(Stay2Out_test()==1)
      Island.Stay2Out_cnt ++;
    else 
      Island.Stay2Out_cnt = 0;
    if(Island.Stay2Out_cnt>2)//连续三次测到突变点状态改变
    {
      if(Island.State==Left_Island_in)
        Island.State = Left_Island_out;
      else if(Island.State==Right_Island_in)
        Island.State = Right_Island_out;
    }
  }

  return 0;
}

u8 Out_Island(void)
{
  int Start_End, End_End;
  int center_impulse;//存储突变点列数
  int center_use;
  static u8  center_Period = 0;
  static u16 Center_[Island_Center_Period_Const];
  
  //检测出环岛标志
  if(Island.State!=Right_Island_out&&Island.State!=Left_Island_out)
    return 1;
  
  center_impulse = Out_Island_Test(&Start_End,&End_End);
  if(Island.State==Right_Island_out)//补线
  {
    center_use = ((center_impulse - (center_impulse - 0)*(End_End - Start_End)*1.0/(End_End - Start_Point)) + 319)/2;
    if(center_use>Image_lie.Three_Lie[1]+30)//有效性检验
    {
      CenterlineToDiff(center_use);
      if(center_Period > (Island_Center_Period_Const - 1))
        center_Period = 0;
    
      Island.Stay_Center  -= Center_[center_Period];
      Center_[center_Period] = Image_hang.center[Image_hang.hang_use];
      Island.Stay_Center  += Center_[center_Period];
      center_Period++;  
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Island.Stay_Center/Island_Center_Period_Const);
    }
  }
  else if(Island.State==Left_Island_out)
  {
    center_use = ((center_impulse - (center_impulse - 319)*(End_End - Start_End)*1.0/(End_End - Start_Point)) + 0)/2;
    if(center_use<Image_lie.Three_Lie[1]-30)//有效性检验
    {
      CenterlineToDiff(center_use);
      if(center_Period > (Island_Center_Period_Const - 1))
        center_Period = 0;
    
      Island.Stay_Center  -= Center_[center_Period];
      Center_[center_Period] = Image_hang.center[Image_hang.hang_use];
      Island.Stay_Center  += Center_[center_Period];
      center_Period++;  
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Island.Stay_Center/Island_Center_Period_Const);
    }
  }
  return 0;
}

u8 Image_Island_Test(void)//捕捉黑线  
{
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int Diff_L[9],Diff_R[9];//一阶差分
  int DDiff_L[8],DDiff_R[8];//二阶差分
  int   Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u8    Liner_L_flag = 0,Liner_R_flag = 0;
  u8    Impulse_L_Flag = 0,Impulse_R_Flag = 0;//一阶差分中出现阶跃
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  for(i=0;i<10;i++)//10行
  {
    ImageData_in = Image_fire[i*3+Island.Image_Start_hang];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    
    Right_Count = Image_lie.Three_Lie[1];//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count+3]==1 
            && ImageData[Right_Count+2]==1
              && ImageData[Right_Count+1]==1)
          && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    if(Right_Count<ccd_end)//如果在有效范围内
    {
      Island.black_R[i] = Right_Count;
    }
    else if(Right_Count<Image_lie.Three_Lie[1]+10)
    {
      Island.black_R[i] = ccd_end;
    }
    else
    {
      Island.black_R[i] = ccd_end;
    }
    Left_Count = Image_lie.Three_Lie[1];
    while(!(ImageData[Left_Count-3]==1 
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      Island.black_L[i] = Left_Count; 
    }
    else if(Left_Count>Image_lie.Three_Lie[1]-10)
    {
      Island.black_L[i] = ccd_start;
    }
    else
    {
      Island.black_L[i] = ccd_start;
    }
  }
  for(i=0;i<9;i++)
  {
    Diff_L[i] = Island.black_L[i+1] - Island.black_L[i];
    Diff_R[i] = Island.black_R[i+1] - Island.black_R[i];
  }
  for(i=0;i<8;i++)
  {
    DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
    DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
    if(Abs_(DDiff_L[i])<3)Liner_L_cnt++;
    if(Abs_(DDiff_R[i])<3)Liner_R_cnt++;
    if(DDiff_L[i]<-30&&Liner_L_cnt>i-2&&Liner_L_cnt>1)
    {
      Impulse_L_Flag=1;//出现冲激
    }
    if(DDiff_R[i]>30&&Liner_R_cnt>i-2&&Liner_R_cnt>1)
    {
      Impulse_R_Flag=1;
    }
  }
  if(Liner_L_cnt>6)Liner_L_flag = 1;
  if(Liner_R_cnt>6)Liner_R_flag = 1;
  
  if(Liner_L_flag&&Impulse_R_Flag==1)
    return Right_Island_pre;
  else if(Liner_R_flag&&Impulse_L_Flag==1)
    return Left_Island_pre;
  else 
    return 0;
}


int Test_Far_Lie(void)//在入环岛时找最远点所在的列，从此列开始向两边寻找突变点
{
  u8 Far_Lie[20];
  u8 Temp_point;
  u8 i;
  for(i=0;i<20;i++)
  {
    Temp_point=180;
    while(!(Image_Point(Temp_point,65+i*10)==1
          &&Image_Point(Temp_point-1,65+i*10)==1
            &&Image_Point(Temp_point-2,65+i*10)==1)&&Temp_point>=10)
    Temp_point--;
    Far_Lie[i] = Temp_point;
  }
  return min_u8_index(Far_Lie,20)*10+65;
}

int Out_Island_Test(int* start_end, int* end_end)//开始计数列的终点和末尾列的终点
{
  u8 Far_Lie[25];
  int Diff_Far_Lie[24];//一阶差分
  int DDiff_Far_Lie[23];//二阶差分
  int Liner_cnt  = 0;
  int out_center = 160;
  
  u8 Temp_point;
  u8 i;
  if(Island.State==Right_Island_out)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=220;
      while(!(Image_Point(Temp_point,0+i*4)==1
            &&Image_Point(Temp_point-1,0+i*4)==1
              &&Image_Point(Temp_point-2,0+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<24;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<23;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<3)//线性标志
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>(i-2)&&Liner_cnt>1)//之前线性，出现冲激
      {
        break;
      }
    }
    out_center = 4*i;//记录突变点
  }
  else if(Island.State==Left_Island_out)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=220;
      while(!(Image_Point(Temp_point,319-i*4)==1
            &&Image_Point(Temp_point-1,319-i*4)==1
              &&Image_Point(Temp_point-2,319-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<24;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<23;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<3)//线性标志
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>(i-2)&&Liner_cnt>1)//之前线性，出现冲激
      {
        break;
      }
    }
    out_center = 319-4*i;//记录突变点
  }
  *start_end = Far_Lie[0];
  *end_end   = Far_Lie[i];
  return out_center;
}

int Stay2Out_test()
{
  u8 Far_Lie[20];
  int Diff_Far_Lie[19];//一阶差分
  int DDiff_Far_Lie[18];//二阶差分
  int Liner_cnt  = 0;
  u8 Impulse_flag = 0;
  
  u8 Temp_point;
  u8 i;
  if(Island.State==Right_Island_in)
  {
    for(i=0;i<20;i++)
    {
      Temp_point=180;
      while(!(Image_Point(Temp_point,30+i*4)==1
            &&Image_Point(Temp_point-1,30+i*4)==1
              &&Image_Point(Temp_point-2,30+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<19;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<18;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<3)//线性标志
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>(i-2)&&Liner_cnt>1)//之前线性，出现冲激
      {
        Impulse_flag = 1;
        break;
      }
    }
  }
  else if(Island.State==Left_Island_in)
  {
    for(i=0;i<20;i++)
    {
      Temp_point=180;
      while(!(Image_Point(Temp_point,289-i*4)==1
            &&Image_Point(Temp_point-1,289-i*4)==1
              &&Image_Point(Temp_point-2,289-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<19;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<18;i++)
    {
      DDiff_Far_Lie[i] = Diff_Far_Lie[i+1] - Diff_Far_Lie[i];
      if(Abs_(DDiff_Far_Lie[i])<3)//线性标志
        Liner_cnt++;
      if(DDiff_Far_Lie[i]<-15&&Liner_cnt>(i-2)&&Liner_cnt>1)//之前线性，出现冲激
      {
        Impulse_flag = 1;
        break;
      }
    }
  }
  return Impulse_flag;
}