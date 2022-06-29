//------------------------------9axis sensor------------------------------
// I2C address for BMX055 Gyro
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// I2C address for BMX055 Magnetic
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;

//Constant for calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
float Calib = 175;
float calibx = 20;
float caliby = 132;

void BMX055_Init();
void BMX055_Gyro();
void BMX055_Mag();
