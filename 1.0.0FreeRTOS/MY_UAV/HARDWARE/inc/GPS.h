#ifndef __GPS_H
#define __GPS_H

#include "sys.h"

typedef struct
{
  float utcHour;
  float utcMin;
  float utcSec;
  float longitude; //����
  float latitude; //γ��
  float gpsSpeed; //GPS�õ����ٶ� ��λ����
  float gpsHeading; //GPS�õ��ķ�λ��
}gpsInfo;


#endif
