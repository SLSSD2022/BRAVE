#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
unsigned int DATA_ADDRESS = 0; //書き込むレジスタ(0x0000~0xFFFF全部使える) 

TinyGPSPlus gps;
SoftwareSerial mySerial(12, 13); // RX, TX
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);
 
void setup() {
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  // mySerial.println("Hello, world?");
}
 
void loop() { // run over and over
  while (mySerial.available() > 0){
    char c = mySerial.read();
    //Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()){
      // Serial.print("LAT="); 
      EEPROM_write_float(DEVICE_ADDRESS, DATA_ADDRESS, gps.location.lat());
      DATA_ADDRESS += 4;
      Serial.print(gps.location.lat(), 10);
      // Serial.print("LONG="); 
      Serial.print(","); 
      Serial.println(gps.location.lng(), 10);
      EEPROM_write_float(DEVICE_ADDRESS, DATA_ADDRESS, gps.location.lng());
      DATA_ADDRESS += 4;
    }
  }
}


void writeEEPROM(int addr_device, unsigned int addr_res, byte data ) 
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}

void EEPROM_write_float(int addr_device, unsigned int addr_res, double data){
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
//    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res+i, p[i]);
  }
//  Serial.println("");
}

byte readEEPROM(int addr_device, unsigned int addr_res ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(addr_device,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


double EEPROM_read_float(int addr_device, unsigned int addr_res){
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res+i);
//    Serial.println(p_read[i]);
  }
//  Serial.println("");
  double *d = (double *)p_read;
  double data = *d;
  return data;
}
