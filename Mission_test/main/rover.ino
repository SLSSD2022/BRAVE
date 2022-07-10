#include "./rover.h"

void _dataStruct::printAll()
{
  Serial.print(":cm_LIDAR:");
  Serial.print(cmLidar);
  Serial.print(":latA:");
  Serial.print(latA);
  Serial.print(":lngA:");
  Serial.print(lngA);
  Serial.print(":latR:");
  Serial.print(latR);
  Serial.print(":lngR:");
  Serial.print(lngR);
  Serial.print(":rangeRtoA:");
  Serial.print(rangeRtoA);
  Serial.print(":degRtoA:");
  Serial.print(degRtoA);
  Serial.print(":x:");
  Serial.print(x);
}

void _modeStruct::printAll()
{
  Serial.println("");
  Serial.println("--------------Rover Mode--------------");
  Serial.print("manual:");
  Serial.println(this->manual);
  Serial.print("autoGPSonly:");
  Serial.println(this->autoGpsOnly);
  Serial.print("autoAggressive:");
  Serial.println(this->autoAggressive);
  Serial.print("sleep:");
  Serial.println(this->sleep);
}

void _statusStruct::printAll()
{
  Serial.println("");
  Serial.println("--------------Rover Status--------------");
  Serial.print("landed:");
  Serial.println(this->landed);
  Serial.print("separated:");
  Serial.println(this->separated);
  Serial.print("evacuated:");
  Serial.println(this->evacuated);
  Serial.print("GPSreceived:");
  Serial.println(this->GPSreceived);
  Serial.print("calibrated:");
  Serial.println(this->calibrated);
  Serial.print("toGoal:");
  Serial.println(this->toGoal);
  Serial.print("near:");
  Serial.println(this->near);
  Serial.print("search:");
  Serial.println(this->search);
  Serial.print("sleep:");
  Serial.println(this->sleep);
}

void _successStruct::printAll()
{
  Serial.println("");
  Serial.println("--------------Rover Success--------------");
  Serial.print("GPSreceive:");
  Serial.println(this->GPSreceive);
  Serial.print("goalGPS:");
  Serial.println(this->goalGPS);
  Serial.print("goalArrived:");
  Serial.println(this->goalArrived);
  Serial.print("full:");
  Serial.println(this->full);
}

void Rover::printAll()
{
  data.printAll();
  mode.printAll();
  status.printAll();
  success.printAll();
}
