#include <SD.h>

// この値は使用しているシールドや基板に合わせて変更すること。たとえば、
// イーサーネットシールドは 4
// Adafruit のSDシールドは 10
// Sparkfun のSDシールドは 8
const int chipSelect = 4;

void setup()
{
  // シリアルポート初期化
  Serial.begin(9600);
  while (!Serial) {
    ; // USBケーブルが接続されるのを待つ。この待ちループは Leonardo のみ必要。
  }

  Serial.print(F("Initializing SD card..."));

  // SSピン（Unoは10番、Megaは53番）は使わない場合でも出力にする必要があります。
  // そうしないとSPIがスレーブモードに移行し、SDライブラリが動作しなくなります。
  pinMode(SS, OUTPUT);

  // SDライブラリを初期化
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // 失敗、何もしない
    while(1);
  }
  Serial.println(F("ok."));

  // ファイルを開く
  File dataFile = SD.open("datalog.txt");

  // もしファイルが開けたら値をシリアルポートに出力する
  if (dataFile) {
    // 64byte単位で読み出す
    byte buffer[64];
    while (dataFile.available()) {
      int length = dataFile.available();
      if(length > 64){
        length = 64;
      }
      dataFile.read(buffer, length);
      Serial.write(buffer, length);
    }
    // 1byte単位で読み出す
    // while (dataFile.available()) {
    // Serial.write(dataFile.read());
    // }
    dataFile.close();
  }
  // ファイルが開けなかったらエラーを出力する
  else {
    Serial.println(F("error opening datalog.txt"));
  }
}

void loop()
{
}
