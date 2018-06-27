#ifndef __MYPWM_H__ 
#define __MYPWM_H__

#include "common.h"
#include "MKL_gpio.h"
#include "MKL_TPM.h"
#include "define.h"
#include "include.h"


void PWM_DISENABLE(TPMn_e tpmn, TPM_CHn_e ch);
void Motot2_Init(void);
void Motot1_Init(void);
void Steer_Init(void);

#endif
