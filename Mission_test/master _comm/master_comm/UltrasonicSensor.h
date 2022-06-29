//------------------------------Ultrasonic sensor------------------------------
//Ultrasonic sensor(short)Front
unsigned int obstacleDistance;
const int HEAD_Trig = 22;
const int HEAD_Echo = 24;

//Ultrasonic sensor(long)Front
#define HEADpin A15
unsigned int anVolt;
unsigned int cm_long;
int emergencyStopDist = 10;

//Buffer for one range measurement near goal
#define MEAS_BUF_LEN  10//it should be more than 10, length of 9axis basic buffer
int bufcm[MEAS_BUF_LEN];
int measureIndex = 0;

//Buffer for all range measurement near goal
#define SEAR_BUF_LEN 20
int listcm[SEAR_BUF_LEN];
int searchIndex = 0;


//Ultrasonic sensor(short)Bottom
const int BOTTOM_Trig = 6;
const int BOTTOM_Echo = 7;

unsigned int getUltrasonic_HEAD();
unsigned int microsecTocm(long microsec);
