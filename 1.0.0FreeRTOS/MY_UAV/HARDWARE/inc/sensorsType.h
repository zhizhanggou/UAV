#ifndef _SENSORSTYPE_H
#define _SENSORSTYPE_H
#include "sys.h"
typedef struct 
{
   float x;
   float y;
   float z;
}Axis3f;


typedef struct 
{
   int16_t x;
   int16_t y;
   int16_t z;
}Axis3i16;

typedef struct 
{
   Axis3f variance;
   Axis3f value;
}GyroBias;


#endif
