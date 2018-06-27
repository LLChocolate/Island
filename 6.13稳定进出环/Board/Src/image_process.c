#include "image_process.h"
#include "include.h"
#include "stdlib.h"
u8 ImageData[320];
Edge_Str Island_mark_edge[50];
float island_addline_k = 0;
float Cur_error = 0;

u8 diff_done_flag = 0;

u16 Start_line_delay = 0xffff;
u8  Start_line_flag  = 0;
u8  Start_line_cnt   = 0;

u8  road_filter_flag = 0;
Filter_1st_Str Center_Filter = {0.8,{0,0},{0,0}};
//Kalman_Date Center_Filter={0,0,0,0,0,0.1,40,0,0};
s16 fangcha_test[200]={0};

u8 Cross_flag;
u16 Cross_flag_delay;
u16 Cross_flag_delay_const = 1000;


Image_hangData Image_hang;
Image_lieData  Image_lie={{78,158,238},{0}};
Island_Data    Island={
                  .Correct_hang = 150,
                  .Stay_hang_use = 127,//这个行是瞎给的
                  .Image_Start_hang = 86,
                  .Next_Island_flag_delay_const = 2000,
                  .Stay2Out_flag_delay_const = 800
                    };
Cross_Data     Cross={
                  .Test_hang = 180
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
  Island_process();
  Cross_process();
    Slow_Flag=0;
    if(Island.State!=NoIsland)
    {
      
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
      case Left_Wait_Next:
        LCD_PutString(250,220,"LW",Red,White);
        break;
      case Right_Wait_Next:
        LCD_PutString(250,220,"RW",Red,White);
        break;
      default:
        break;
      }
    }
    DIFF_PID_CHANGE_FLAG=0;//使用直道PID
}

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
}


u8 CenterlineToDiff(int center)
{
  int real_center;
  if(diff_done_flag == 1)return 0;
  

//*********************对中心点均值滤波*************************************  
  
  real_center = filter_1st(center,&Center_Filter);
  diff_done_flag = 1;
  if(Island.State==Left_Island_pre||Island.State==Right_Island_pre)
    Diff_error = 160 - real_center;
  else
    Diff_error = 160 - center;
  
  return 0;
}


u8 Str_Cross_Test(void)
{
  u8 i,cnt=0;
  for(i=0;i<5;i++)
  {
    if(sum_point(Image_fire[Start_Point+i*2],40)<=5)
      cnt++;
  }
  if(cnt>3&&Image_lie.Three_lie_end[0]<120&&Image_lie.Three_lie_end[2]<120)//近处没有尖角
  {
    return 1;
  }
  else 
    return 0;
}

u8 double_AD(void)
{
  if((L_AD_Ave>3000)||(R_AD_Ave>2400))
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

u8 Island_process(void)
{
  Elec_Island();//电磁检测，只在入环和出环的时候检测
  In_Island();//入环岛
  Stay_Island();//在环岛里
  Out_Island();//出环岛
  Wait_Next_Island();//防止再次进入环岛
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
  
  if(Island.State == Left_Island_out)
  {
    if(double_AD()==1)
    {
      Island.State = Left_Wait_Next;//等待下一个环岛的时间间隔
      Island.Next_Island_flag = 1;
      Island.Next_Island_flag_delay = Island.Next_Island_flag_delay_const;
    }
  }
  else if(Island.State == Right_Island_out)
  {
    if(double_AD()==1)
    {
      Island.State = Right_Wait_Next;//等待下一个环岛的时间间隔
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
        Beep_Once(&Image_Island_Test_Beep);
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
          if(Impulse_hang<95)
            center_use = ((center - (center - 319)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 0)/2 + 50;//布线（三角形相似）
          else
            center_use = ((center - (center - 319)*(Impulse_hang - Start_Point)*1.0/(Impulse_hang - Island.Correct_hang)) + 0)/2 + 30;//布线（三角形相似）
          if(center_use<35)
            center_use = 35;
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
          if(center_use>285)
            center_use = 285;
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
      Left_Count = Middle;
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
  
  if(Island.State == Right_Island_pre)
  {
    for(i=0;i<19;i++)
    {
      Diff_R[i] = In_black_R[i+1] - In_black_R[i];
      if(Diff_R[i]>30)
      {
        Impulse_R_Flag = 1;
        center = In_black_R[i];//出现跳转的行
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
  }
  else if(Island.State == Left_Island_pre)
  {
    for(i=0;i<19;i++)
    {
      Diff_L[i] = In_black_L[i+1] - In_black_L[i];
      if(Diff_L[i]<-30)
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

u8 Stay_Island(void)
{
  if(Island.State!=Left_Island_in&&Island.State!=Right_Island_in)
    return 1;
  
//当作普通弯道寻找中线
  get_black_line(Image_fire[Island.Stay_hang_use],Island.Stay_hang_use);//
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
    if(Island.Stay2Out_cnt>1)//连续两次测到突变点状态改变
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
    center_use = ((center_impulse - (center_impulse - 0)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 319)/2;
    if(center_use>Image_lie.Three_Lie[1]+30)//有效性检验
    {
      if(center_use > 285)
        center_use = 285;
      CenterlineToDiff(center_use);
      if(center_Period > (Island_Center_Period_Const - 1))
        center_Period = 0;
    
      Island.Out_Center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Island.Out_Center  += Center_[center_Period];
      center_Period++;  
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//行列颠倒
      }
    }
    else if((Image_lie.Three_lie_end[0]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[2])
    {
      //当作普通弯道寻找中线
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]>170)//检验中点正确性
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Island.Out_Center/Island_Center_Period_Const,Start_Point,Magenta);//行列颠倒
      }
    }
  }
  else if(Island.State==Left_Island_out)
  {
    center_use = ((center_impulse - (center_impulse - 319)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 0)/2;
    if(center_use<Image_lie.Three_Lie[1]-30)//有效性检验
    {
      if(center_use < 35)
        center_use = 35;
      CenterlineToDiff(center_use);
      if(center_Period > (Island_Center_Period_Const - 1))
        center_Period = 0;
    
      Island.Out_Center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Island.Out_Center  += Center_[center_Period];
      center_Period++;  
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//行列颠倒
      }
    }
    else if((Image_lie.Three_lie_end[2]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[0])
    {
      //当作普通弯道寻找中线
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]<150)//检验中点正确性
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Island.Out_Center/Island_Center_Period_Const);
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Island.Out_Center/Island_Center_Period_Const,Start_Point,Magenta);//行列颠倒
      }
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
    if(Abs_(DDiff_L[i])<3&&Abs_(Diff_L[i])>0)
      Liner_L_cnt++;
    if(Abs_(DDiff_R[i])<3&&Abs_(Diff_R[i])>0)
      Liner_R_cnt++;
    if(DDiff_L[i]<-30
       &&Island.black_L[i]>70
       &&Liner_L_cnt>i-3
       &&Liner_L_cnt>0)
    {
      Impulse_L_Flag=1;//出现冲激
    }
    if(DDiff_R[i]>30
       &&Island.black_R[i]<249
       &&Liner_R_cnt>i-3
       &&Liner_R_cnt>0)
    {
      Impulse_R_Flag=1;
    }
  }
  if(Liner_L_cnt>5)Liner_L_flag = 1;
  if(Liner_R_cnt>5)Liner_R_flag = 1;
  
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
      Temp_point=239;
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
      Temp_point=239;
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
  u8 Far_Lie[30];
  int Diff_Far_Lie[29];//一阶差分
  int DDiff_Far_Lie[28];//二阶差分
  int Liner_cnt  = 0;
  u8 Impulse_flag = 0;
  
  u8 Temp_point;
  u8 i;
  if(Island.State==Right_Island_in)
  {
    for(i=0;i<30;i++)
    {
      Temp_point=239;
      while(!(Image_Point(Temp_point,30+i*4)==1
            &&Image_Point(Temp_point-1,30+i*4)==1
              &&Image_Point(Temp_point-2,30+i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<29;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<28;i++)
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
    for(i=0;i<30;i++)
    {
      Temp_point=239;
      while(!(Image_Point(Temp_point,289-i*4)==1
            &&Image_Point(Temp_point-1,289-i*4)==1
              &&Image_Point(Temp_point-2,289-i*4)==1)&&Temp_point>=10)
      Temp_point--;
      Far_Lie[i] = Temp_point;
    }
    for(i=0;i<29;i++)
    {
      Diff_Far_Lie[i] = Far_Lie[i+1] - Far_Lie[i];
    }
    for(i=0;i<28;i++)
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

int Wait_Next_Island()
{
  int center;
  u8 hang_use;
  if(Island.State!=Left_Wait_Next&&Island.State!=Right_Wait_Next)//不在此状态下
    return 1;//直接返回
  
  center = Wait_Next_center(&hang_use);
  if(center==-1)//出环岛时寻找突变点失败
  {
    get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
    if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
       &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
    {
      get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
    }
    CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  }
  else
  {
    get_black_line(Image_fire[hang_use],hang_use);
    CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  }
  return 0;
}


int Wait_Next_center(u8* hang)//出环岛时防止再次进入环岛，寻找突变点
{
  int Middle;
  int center;
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int Diff_L[19],Diff_R[19];//一阶差分
  int DDiff_L[18],DDiff_R[18];//二阶差分
  int Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u16 Next_black_L[20];
  u16 Next_black_R[20];
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
    if(Island.State == Right_Wait_Next)
    {
      Right_Count = Middle;//把黑线中间值赋给右计数起点
      while(!(ImageData[Right_Count+3]==1
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
      if(Right_Count<ccd_end)//如果在有效范围内
      {
        Next_black_R[i] = Right_Count;
      }
      else if(Right_Count<Image_lie.Three_Lie[1]+10)
      {
        Next_black_R[i] = ccd_end;
      }
      else
      {
        Next_black_R[i] = ccd_end;
      }
    }
    else if(Island.State == Left_Wait_Next)
    {
      Left_Count = Image_lie.Three_Lie[1];
      while(!(ImageData[Left_Count-3]==1 
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        Next_black_L[i] = Left_Count; 
      }
      else if(Left_Count>Image_lie.Three_Lie[1]-10)
      {
        Next_black_L[i] = ccd_start;
      }
      else
      {
        Next_black_L[i] = ccd_start;
      }
    }
  }
  for(i=0;i<19;i++)
  {
    if(Island.State == Right_Wait_Next)
    {
      Diff_R[i] = Next_black_R[i+1] - Next_black_R[i];
    }
    else if(Island.State == Left_Wait_Next)
    {
      Diff_L[i] = Next_black_L[i+1] - Next_black_L[i];
    }
  }
  for(i=0;i<18;i++)
  {
    if(Island.State == Right_Wait_Next)
    {
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_R[i])<3)Liner_R_cnt++;
      if(DDiff_R[i]<-30&&Liner_R_cnt>i-3&&Liner_R_cnt>1)
      {
        Impulse_R_Flag = 1;
        center = Next_black_R[i];//出现跳转的行
        *hang   = i*3+Island.Image_Start_hang;
        break;
      }
    }
    else if(Island.State == Left_Wait_Next)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      if(Abs_(DDiff_L[i])<3)Liner_L_cnt++;
      if(DDiff_L[i]> 30&&Liner_L_cnt>i-3&&Liner_L_cnt>1)
      {
        Impulse_L_Flag = 1;//出现冲激
        center = Next_black_L[i];//出现跳转的行
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

u8 Cross_process(void)
{
  if(Island.State!=NoIsland)//环岛优先级最高
    return 1;
  Cross.State = NoCross;//清空状态
  In_Cross_test();//斜入十字检测
  In_Cross();//斜入十字
  Str_Cross();
  return 0;
}

u8 In_Cross_test()//斜入十字检测
{
  u8 Cross_curve_flag = 0;
  
  if(Cross.State==R2Cross_True
     ||Cross.State==L2Cross_True
     ||Cross.State==Str2Cross)
  return 1;
  
  Cross_pre_test();
  Cross_curve_flag = Cross_curve_test();
//  if(Cross_curve_flag==0)//不是斜入十字
//  {
//    if(Cross_Test()==1)//优先级最低
//    Cross.State = Str2Cross;
//  }
  return 0;
}

u8 In_Cross(void)//斜入十字
{
  int Start_End, End_End;
  int center_impulse;//存储突变点列数
  int center_use;
  static u8  center_Period = 0;
  static u16 Center_[Cross_Center_Period_Const];
  
  //检测斜入十字标志
  if(Cross.State!=R2Cross_True&&Cross.State!=L2Cross_True)
    return 1;
  
  center_impulse = Cross_center_test(&Start_End,&End_End);
  if(Cross.State==R2Cross_True)//补线
  {
    center_use = ((center_impulse - (center_impulse - 0)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 319)/2;
    if(center_use>Image_lie.Three_Lie[1]+30)//有效性检验
    {
      if(center_use > 285)
        center_use = 285;
      CenterlineToDiff(center_use);
      if(center_Period > (Cross_Center_Period_Const - 1))
        center_Period = 0;
    
      Cross.In_center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Cross.In_center  += Center_[center_Period];
      center_Period++;  
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//行列颠倒
      }
    }
    else if((Image_lie.Three_lie_end[0]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[2])
    {
      //当作普通弯道寻找中线
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]>170)//检验中点正确性
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Cross.In_center/Cross_Center_Period_Const,Start_Point,Magenta);//行列颠倒
      }
    }
  }
  else if(Cross.State==L2Cross_True)
  {
    center_use = ((center_impulse - (center_impulse - 319)*(End_End - Start_Point)*1.0/(End_End - Start_End)) + 0)/2;
    if(center_use<Image_lie.Three_Lie[1]-30)//有效性检验
    {
      if(center_use < 35)
        center_use = 35;
      CenterlineToDiff(center_use);
      if(center_Period > (Cross_Center_Period_Const - 1))
        center_Period = 0;
    
      Cross.In_center  -= Center_[center_Period];
      Center_[center_Period] = center_use;
      Cross.In_center  += Center_[center_Period];
      center_Period++;  
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(center_use,Start_Point,Cyan);//行列颠倒
      }
    }
    else if((Image_lie.Three_lie_end[2]-10)>Image_lie.Three_lie_end[1]
            ||(Image_lie.Three_lie_end[1]-10)>Image_lie.Three_lie_end[0])
    {
      //当作普通弯道寻找中线
      get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
      if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
           &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
      {
        get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
      }
      if(Image_hang.center[Image_hang.hang_use]<150)//检验中点正确性
        CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
      else
        CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
    }
    else//补线失败，使用之前保存的中心点
    {
      CenterlineToDiff(Cross.In_center/Cross_Center_Period_Const);
//调试用，显示中心点
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_DrawBigPoint(Cross.In_center/Cross_Center_Period_Const,Start_Point,Magenta);//行列颠倒
      }
    }
  }
  return 0;
}

u8 Cross_curve_test()
{
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int L_black[60],R_black[60];//左右边界
  int Diff_L[59],Diff_R[59];//一阶差分
  int DDiff_L[58],DDiff_R[58];//二阶差分
  int   Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u8    Liner_L_flag = 0,Liner_R_flag = 0;
  u8    Turn_L_Flag = 0,Turn_R_Flag = 0;//出现转折点
  u8    Turn_L_index= 0,Turn_R_index= 0;//出现转折点的位置
  int   Turn_L_early_cnt = 0,Turn_R_early_cnt = 0;//出现转折点之前边沿捕获
  int   Turn_L_late_cnt = 0 ,Turn_R_late_cnt = 0 ;//出现转折点之后边沿捕获
  u8  Impulse_L_Flag = 0,Impulse_R_Flag = 0;//一阶差分中出现阶跃
  int Impulse_L_index = 0,Impulse_R_index = 0;//一阶差分出现阶跃的位置
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  for(i=0;i<60;i++)//10行
  {
    ImageData_in = Image_fire[Cross.Test_hang-i*2];
    for(j=0;j<40;j++)
      for(u8 k=0;k<8;k++)
        ImageData[j*8+k] = (ImageData_in[j]>>(7-k))&0x01;
    
    if(Cross.State==L2Cross_Pre)//左转找右边界
    {
      Right_Count = 35;
      while(!(ImageData[Right_Count+3]==1
              && ImageData[Right_Count+2]==1
                && ImageData[Right_Count+1]==1)
            && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
      if(Right_Count< ccd_end)//如果在有效范围内
      {
        R_black[i] = Right_Count;
      }
      else
      {
        R_black[i] = ccd_end;
      }
    }
    else if(Cross.State==R2Cross_Pre)//左转找右边界
    {
      Left_Count = 0;
      while(!(ImageData[Left_Count-3]==1
              && ImageData[Left_Count-2]==1
                && ImageData[Left_Count-1]==1)
            && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count > ccd_start)
      {
        L_black[i] = Left_Count; 
      }
      else
      {
        L_black[i] = ccd_start;
      }
    }
  }
  if(Cross.State==L2Cross_Pre)
  {
    for(i=0;i<59;i++)
    {
      Diff_R[i] = R_black[i+1] - R_black[i];
      if(Diff_R[i]<-100)
      {
        Impulse_R_Flag = 1;
        Impulse_R_index = i;
      }
    }
    for(i=0;i<59;i++)
    {
      if(Turn_R_Flag==0&&Diff_R[i]<=0)//未出现转折点
        Turn_R_early_cnt++;
      else if(Turn_R_Flag==0
              &&i<50            //防止内存溢出
              &&Diff_R[i]>0
              &&Diff_R[i+1]>0
              &&Diff_R[i+2]>0)//出现转折点
      {
        Turn_R_Flag = 1;
        Turn_R_late_cnt++;
        Turn_R_index = i;//找到转折点的位置
      }
      else if(Turn_R_Flag==1&&Diff_R[i]>0)
      {
        Turn_R_late_cnt++;
      }
    }
    
  }
  else if(Cross.State==R2Cross_Pre)
  {
    for(i=0;i<59;i++)
    {
      Diff_L[i] = L_black[i+1] - L_black[i];
      if(Diff_L[i]>100)
      {
        Impulse_L_Flag = 1;
        Impulse_L_index = i;
      }
    }
    for(i=0;i<59;i++)
    {
      if(Turn_L_Flag==0&&Diff_L[i]>=0)//未出现转折点
        Turn_L_early_cnt++;
      else if(Turn_L_Flag==0
              &&i<50            //防止内存溢出
              &&Diff_L[i]<0
              &&Diff_L[i+1]<0
              &&Diff_L[i+2]<0)//出现转折点
      {
        Turn_L_Flag = 1;
        Turn_L_late_cnt++;
        Turn_L_index = i;//找到转折点的位置
      }
      else if(Turn_L_Flag==1&&Diff_L[i]<0)
      {
        Turn_L_late_cnt++;
      }
    }
  }
  if(Cross.State==L2Cross_Pre)
  {
    for(i=0;i<Turn_R_index;i++)
    {
      DDiff_R[i] = Diff_R[i+1] - Diff_R[i];
      if(Abs_(DDiff_R[i])<3)Liner_R_cnt++;
    }
    if(Liner_R_cnt>Turn_R_index*2/3)Liner_R_flag = 1;
  }
  else if(Cross.State==R2Cross_Pre)
  {
    for(i=0;i<Turn_L_index;i++)
    {
      DDiff_L[i] = Diff_L[i+1] - Diff_L[i];
      if(Abs_(DDiff_L[i])<3)Liner_L_cnt++;
    }
    if(Liner_L_cnt>Turn_L_index*2/3)Liner_L_flag = 1;
  }
  if(Cross.State==L2Cross_Pre)
  {
    if(Liner_R_flag
       &&Turn_R_Flag
       &&Turn_R_index>5
       &&Turn_R_early_cnt>Turn_R_index*2/3
       &&Turn_R_index<45
       &&Impulse_R_Flag
       &&Turn_R_late_cnt>(Impulse_R_index-Turn_R_index)*2/3
         )
    {
      Cross.State = L2Cross_True;
//      Beep_Once(&Image_Island_Test_Beep);
      return 1;
    }
  }
  else if(Cross.State==R2Cross_Pre)
  {
    if(Liner_L_flag
       &&Turn_L_Flag
       &&Turn_L_index>5
       &&Turn_L_early_cnt>Turn_L_index*2/3
       &&Turn_L_index<45
       &&Impulse_L_Flag
       &&Turn_L_late_cnt>(Impulse_L_index-Turn_L_index)*2/3
         )
    {
      Cross.State = R2Cross_True;
//      Beep_Once(&Image_Island_Test_Beep);
      return 1;
    }
  }
  return 0;
}

u8 Cross_center_test(int* start_end, int* end_end)//和出环岛时找中心点的代码一样
{
  u8 Far_Lie[25];
  int Diff_Far_Lie[24];//一阶差分
  int DDiff_Far_Lie[23];//二阶差分
  int Liner_cnt  = 0;
  int out_center = 160;
  
  u8 Temp_point;
  u8 i;
  if(Cross.State==R2Cross_True)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=239;
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
  else if(Cross.State==L2Cross_True)
  {
    for(i=0;i<25;i++)
    {
      Temp_point=239;
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

u8 Out_Cross_test(void)
{
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int L_black[10],R_black[10];
  int Diff_L[9],Diff_R[9];//一阶差分
  int DDiff_L[8],DDiff_R[8];//二阶差分
  int   Liner_L_cnt  = 0,Liner_R_cnt  = 0;
  u8    Liner_L_flag = 0,Liner_R_flag = 0;
  u8    Impulse_L_Flag = 0,Impulse_R_Flag = 0;//一阶差分中出现阶跃
  u8 i = 0,j = 0;
  u8 *ImageData_in;
  
  if(Cross.State!=L2Cross_True&&Cross.State!=R2Cross_True)
    return 1;
  
  for(i=0;i<10;i++)//10行
  {
    ImageData_in = Image_fire[i*3+Far_Point];
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
      R_black[i] = Right_Count;
    }
    else
    {
      R_black[i] = ccd_end;
    }
    Left_Count = Image_lie.Three_Lie[1];
    while(!(ImageData[Left_Count-3]==1 
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      L_black[i] = Left_Count; 
    }
    else
    {
      L_black[i] = ccd_start;
    }
  }
  for(i=0;i<9;i++)
  {
    Diff_L[i] = L_black[i+1] - L_black[i];
    Diff_R[i] = R_black[i+1] - R_black[i];
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
  
  if(  Liner_L_flag
     &&Impulse_R_Flag
     &&Liner_R_flag
     &&Impulse_L_Flag)
  {
    Cross.State = NoCross;
    return 0;
  }
}


u8 Cross_pre_test(void)
{
  u8 Temp_point;
  u8 i;
  int L_Far_Lie[10],R_Far_Lie[10];
  int L_Diff_Far_Lie[9],R_Diff_Far_Lie[9];//一阶差分
  int L_DDiff_Far_Lie[8],R_DDiff_Far_Lie[8];//二阶差分
  int L_Liner_cnt = 0,R_Liner_cnt = 0;
  
  for(i=0;i<10;i++)
  {
    Temp_point=239;
    while(!(Image_Point(Temp_point,0+i*3)==1
          &&Image_Point(Temp_point-1,0+i*3)==1
            &&Image_Point(Temp_point-2,0+i*3)==1)&&Temp_point>=130)
    Temp_point--;
    L_Far_Lie[i] = Temp_point;
    
    Temp_point=239;
    while(!(Image_Point(Temp_point,319-i*3)==1
          &&Image_Point(Temp_point-1,319-i*3)==1
            &&Image_Point(Temp_point-2,319-i*3)==1)&&Temp_point>=130)
    Temp_point--;
    R_Far_Lie[i] = Temp_point;
  }
  for(i=0;i<9;i++)
  {
    L_Diff_Far_Lie[i] = L_Far_Lie[i+1] - L_Far_Lie[i];
    R_Diff_Far_Lie[i] = R_Far_Lie[i+1] - R_Far_Lie[i];
  }
  for(i=0;i<8;i++)
  {
    L_DDiff_Far_Lie[i] = L_Diff_Far_Lie[i+1] - L_Diff_Far_Lie[i];
    R_DDiff_Far_Lie[i] = R_Diff_Far_Lie[i+1] - R_Diff_Far_Lie[i];
    if(Abs_(L_DDiff_Far_Lie[i])<3&&Abs_(L_Diff_Far_Lie[i])>0)//线性标志
      L_Liner_cnt++;
    if(Abs_(R_DDiff_Far_Lie[i])<3&&Abs_(R_Diff_Far_Lie[i])>0)//线性标志
      R_Liner_cnt++;
  }
  if(L_Liner_cnt>4&&R_Liner_cnt>4)
  {
    return 1;
  }
  else if(L_Liner_cnt>4)
    Cross.State = R2Cross_Pre;
  else if(R_Liner_cnt>4)
    Cross.State = L2Cross_Pre;
  return 0;
}



u8 Str_Cross(void)
{
  int center_temp;
  if(Cross.State==L2Cross_True||Cross.State==R2Cross_True)
    return 1;
  if(Str_Cross_Test()==1)//直入十字
  {
    center_temp = Test_Far_Lie();
    Image_hang.center[Start_Point] = center_temp;
    get_black_line(Image_fire[Start_Point],Start_Point);//45cm处中心点
    if(Image_lie.Three_lie_end[0]>Image_hang.hang_use+5
       &&Image_lie.Three_lie_end[1]>Image_hang.hang_use+5
         &&Image_lie.Three_lie_end[2]>Image_hang.hang_use+5)//去除光斑的影响
    {
      get_black_line(Image_fire[Image_lie.Three_lie_end[1]+3],Image_lie.Three_lie_end[1]+3);
    }
    CenterlineToDiff(Image_hang.center[Image_hang.hang_use]);
  }
}