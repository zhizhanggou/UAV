#ifndef _IIC_H
#define _IIC_H
#include "sys.h"
//#define IIC_PORT1 GPIO_PIN_3 | GPIO_PIN_4
typedef struct iic{
   
 uint16_t sclPort;
 uint16_t sdaPort;  
 uint16_t iicPortNum1;  
 GPIO_TypeDef * iic_Port;  
   
}iic;

void iicInit(void);


#endif

