#include <Wire.h>

//超音波センサー(短)前面
unsigned int cm;

//超音波センサー(長)前面


//超音波センサー(短)底面


//モーター
int speed_R;
int speed_L;


//9軸センサー
// センサーの値を保存するグローバル変数
float xGyro;
float yGyro;
float zGyro;
int xMag;
int yMag;
int zMag;

//キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calib; 
double Calibx;
double Caliby;

//9軸フィルター関連


//GPS
double LatR;
double LongR;  //現在地

float degRtoA; //GPS現在地における目的地の慣性方角
float x; //ローバーの慣性姿勢
float delta_theta;//目的方向と姿勢の相対角度差

//EEPROM
//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
unsigned int DATA_ADDRESS = 0; //書き込むレジスタ(0x0000~0xFFFF全部使える) 

//制御ステータス・HK関連
boolean GPS_flag;
boolean Stop_flag;
boolean Near_flag;
boolean Success_flag;
int Memory_flag;
int Status_control;



void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  
  //TWElite通信用ハードウェアシリアル
  Serial2.begin(9600);

  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  
  delay(1000);
}

void loop()
{
  //---------------------ログ書き込み------------------------------------------------------
  Serial.print("[ADDRESS:");
  Serial.print(DATA_ADDRESS);
  x = EEPROM_read_float(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 4;
  cm = EEPROM_read_int(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 2;
  LatR = EEPROM_read_float(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 4;
  LongR = EEPROM_read_float(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 4;
  degRtoA = EEPROM_read_float(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 4;
  Status_control = (int)readEEPROM(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 2;
  unsigned long time = EEPROM_read_long(DEVICE_ADDRESS, DATA_ADDRESS);
  DATA_ADDRESS += 4;

  //---------------------ステータス表示--------------------------------------------------
  Serial.print(":time:");
  Serial.print(time);
  Serial.print(":LatR:");
  Serial.print(LatR);
  Serial.print(":LongR:");
  Serial.print(LongR);
  Serial.print(":degRtoA:");
  Serial.print(degRtoA);
  Serial.print(":x:");
  Serial.print(x);
  Serial.print(":cm:");
  Serial.print(cm);
  switch(Status_control){
    case 0:
      Serial.print(":stop!");
      break;
    case 1:
      Serial.print(":Go straight!");
      break;
    case 2:
      Serial.print(":Turn left!");
      break;
    case 3:
      Serial.print(":Turn right!");
      break;
    case 4:
      Serial.print(":Spin to right!");
      break;
    case 5:
      Serial.print(":Spin to left!");
      break;
    case 6:
      Serial.print(":Back!");
      break;
    default:
      Serial.print("Motor Unavialble...");
  }
  
  //最後にシリアル通信を改行する
  Serial.println("");
}





//============EEPROM関連=========================================================================//
void writeEEPROM(int addr_device, unsigned int addr_res, byte data ) 
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
}

void EEPROM_write_int(int addr_device, unsigned int addr_res, int data){
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


int EEPROM_read_int(int addr_device, unsigned int addr_res){
  unsigned char p_read[2];
  for (int i = 0; i < 2; i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res+i);
//    Serial.println(p_read[i]);
  }
//  Serial.println("");
  int *d = (int *)p_read;
  int data = *d;
  return data;
}


unsigned long EEPROM_read_long(int addr_device, unsigned int addr_res){
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res+i);
//    Serial.println(p_read[i]);
  }
//  Serial.println("");
  unsigned long *d = (unsigned long *)p_read;
  unsigned long data = *d;
  return data;
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
