#include "include.h"
#include "common.h"
#include "getspeed.h"

void getspeed1_init()
{
  gpio_init(PTE25,GPI,HIGH);
  tpm_pulse_init(TPM1,TPM_CLKIN0,TPM_PS_1);
}

void getspeed2_init()
{
  gpio_init(PTE26,GPI,HIGH);
  tpm_pulse_init(TPM2,TPM_CLKIN1,TPM_PS_1);
}

void Get_speed1(Motor_Status* Motor)//²âËÙ
{
//  Temp_L_DIR = PTE25_IN;
  Motor->Dir = PTE25_IN;
  if(Motor->Dir == 1)
    Motor->Speed = tpm_pulse_get(TPM1);
  else 
    Motor->Speed = -tpm_pulse_get(TPM1);
  tpm_pulse_clean(TPM1);
}

void Get_speed2(Motor_Status* Motor)//²âËÙ
{
//  Temp_R_DIR = PTE26_IN;
  Motor->Dir = PTE26_IN;
  if(Motor->Dir == 0)
    Motor->Speed= tpm_pulse_get(TPM2);
  else 
    Motor->Speed= -tpm_pulse_get(TPM2);
  tpm_pulse_clean(TPM2);
}