#include <TinyGPSPlus.h>

//------------------------------GPS sensor------------------------------
TinyGPSPlus gps;
// float LatA = 35.7100069, LngA = 139.8108103;  //目的地Aの緯度経度(スカイツリー)
//float LatA = 35.7142738, LngA = 139.76185488809645; //目的地Aの緯度経度(2号館)
//float LatA = 35.7140655517578, LngA = 139.7602539062500; //目的地Aの緯度経度(工学部広場)
float LatA = 35.719970703125, LngA = 139.7361145019531; //目的地Aの緯度経度((教育の森公園)
float latR = 35.715328, lngR = 139.761138;  //現在地の初期想定値(7号館屋上)
float degRtoA; //GPS現在地における目的地の慣性方角
float rangeRtoA;

typedef struct _gpsDataStruct {
  float latA[3];
  float lngA[3];
} gpsDataStruct;

typedef union _gpsPacketUnion {
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsDataStruct)];
} gpsPacketUnion;

void updateGPSSolution();