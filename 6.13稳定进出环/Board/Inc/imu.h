#ifndef _IMU_H_
#define _IMU_H_
#include "include.h"

#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f

void imuUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt,float *roll,float *pitch,float *yaw);	/*Êý¾ÝÈÚºÏ »¥²¹ÂË²¨*/
void Body_To_Earth(float *Earth_x,float *Earth_y,float *Earth_z,float Body_x,float Body_y,float Body_z,float Yaw,float Roll,float Pitch);
void acc_measure_upspeed(void);
void Altitude_Update(u16 Time_Ms,float ACC_Earth_Z,float ultr_get_cm);
#endif



