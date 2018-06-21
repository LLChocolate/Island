#ifndef __SCCB_H
#define __SCCB_H


#define SCL_H()         PTC16_OUT = 1
#define SCL_L()         PTC16_OUT = 0
#define	SCL_DDR_OUT() 	PTC16_DDR = 1
#define	SCL_DDR_IN() 	PTC16_DDR = 0

#define SDA_H()         PTC17_OUT = 1
#define SDA_L()         PTC17_OUT = 0
#define SDA_IN()      	PTC17_IN
#define SDA_DDR_OUT()	PTC17_DDR = 1
#define SDA_DDR_IN()	PTC17_DDR = 0

#define ADDR_OV7725   0x42

#define SCCB_DELAY()	SCCB_delay(100)	


void SCCB_GPIO_init(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u16 length,   u8 ReadAddress);
static void SCCB_delay(u16 i);


#endif 
