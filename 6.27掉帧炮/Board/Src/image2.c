

///*************************************************************************
//*  函数名称：out_island
//*  功能说明：在出环岛的时候检测尖角标志
//*  参数说明：环岛方向
//*  函数返回：是否需要补线
//*  修改时间：2018-4-16    已测试
//*  备    注：//流程：先检测左边黑色地带（尖角）
//              //向上依次寻找黑色地带每行寻找白色边缘
//              //向上找不到黑色地带后向下找黑色边缘
//*************************************************************************/

#include "image2.h"
u8 out_island(u8 Lock_island)
{
  u8 over_flag = 0;
  u8 find_dir = 0;
  u8 edge_j = 0;
  int px = 239,py;
//  u8 finish_flag = 0;
  if(Lock_island == Right_Island)
  {
    py = 0;
    if(Image_Point(239,0)==0)
    {
      find_dir = right;//最左下角是黑色，向右找白色
      py++;
    }
    else 
    {
      while(!(Image_Point(px,py)==1
        &&Image_Point(px-1,py)==1
          &&Image_Point(px-2,py)==1)&&px>70)//找到黑色地带
      px--;
      if(px>70)
        Island_mark_edge[edge_j++].x = px;
      else return 0;//尖角
    }
  }
  else if(Lock_island == Left_Island)
  {
    py = 319;
    if(Image_Point(239,0)==0)
    {
      find_dir = left;//最右下角是黑色，向左找白色
      py--;
    }
    else 
    {
      while(!(Image_Point(px,py)==1
        &&Image_Point(px-1,py)==1
          &&Image_Point(px-2,py)==1)&&px>70)//找到黑色地带
      px--;
      if(px>70)
        Island_mark_edge[edge_j++].x = px;
      else return 0;
    }
  }
  while(px>70)
  {
    if(Image_Point(px,py)==1)//当前为黑色区域
    {
    switch(find_dir)
    {
    case left:
      while(!(Image_Point(px,py)==0
        &&Image_Point(px,py-1)==0
          &&Image_Point(px,py-2)==0)&&py>160)
      py--;
      if(py>160)
      Island_mark_edge[edge_j++].y1 = py;//边缘的y坐标
      else 
//      Island_mark_edge[edge_j].dir |= left;//边缘相对左方
      break;
    case right:
      while(!(Image_Point(px,py)==0
      &&Image_Point(px,py+1)==0
        &&Image_Point(px,py+2)==0)&&py<160)
      py--;
      if(py<160)
      Island_mark_edge[edge_j++].y1 = py;//边缘的y坐标
//      Island_mark_edge[edge_j].dir |= left;//边缘相对左方
      break;
    default:
      break;
    }
      px--;
      Island_mark_edge[edge_j].x = px;
    }
    else
    {
      over_flag = 1;
      island_addline_k = (Island_mark_edge[edge_j].y1-Island_mark_edge[0].y1)/(Island_mark_edge[edge_j].x-Island_mark_edge[0].x);
      break;
    }
  }
  return over_flag;
}

