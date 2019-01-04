#ifndef __GPS_H
#define __GPS_H

#include "sys.h"

typedef struct
{
  float utcHour;
  float utcMin;
  float utcSec;
  float longitude; //经度
  float latitude; //纬度
  float gpsSpeed; //GPS得到的速度 单位：节
  float gpsHeading; //GPS得到的方位角
}gpsInfo;


#endif
