#include "./rover.h"

void _dataStruct::printAll()
{
  Serial.print(":latA:");
  Serial.print(latA);
  Serial.print(":lngA:");
  Serial.print(lngA);
  Serial.print(":latR:");
  Serial.print(latR);
  Serial.print(":lngR:");
  Serial.print(lngR);
  Serial.print(":degRtoA:");
  Serial.print(degRtoA);
  Serial.print(":x:");
  Serial.print(x);
  Serial.print(":rangeRtoA:");
  Serial.print(rangeRtoA);
  Serial.print(":cm_LIDAR:");
  Serial.print(cmLidar);
}

