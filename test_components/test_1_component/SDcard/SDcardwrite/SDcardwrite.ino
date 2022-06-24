#include <SD.h>

// この値は使用しているシールドや基板に合わせて変更すること。たとえば、
// イーサーネットシールドは 4
// Adafruit のSDシールドは 10
// Sparkfun のSDシールドは 8
const int chipSelect = 53;

void setup()
{
  // シリアルポート初期化
  Serial.begin(9600);
  while (!Serial) {
    ; // USBケーブルが接続されるのを待つ。この待ちループは Leonardo のみ必要。
  }

  Serial.print(F("Initializing SD card..."));

  // SSピン（Unoは10番、Megaは53番）は使わない場合でも出力にする必要があります。
  // そうしないと、SPIがスレーブモードに移行し、SDライブラリが動作しなくなります。
  pinMode(SS, OUTPUT);

  // SDライブラリを初期化
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    // 失敗、何もしない
    while(1);
  }
  Serial.println(F("ok."));

  // 日付と時刻を返す関数を登録
  SdFile::dateTimeCallback( &dateTime );
}

void loop()
{
  // ファイルを開く
  File dataFile = SD.open("datalog2.txt", FILE_WRITE);

  // もしファイルが開けたら値を書き込む
  if (dataFile) {
    int value = 7;
    dataFile.println(value);
    dataFile.close();
    // シリアルポートにも出力
    Serial.println(value);
  }
  // ファイルが開けなかったらエラーを出力
  else {
    Serial.println(F("error opening datalog.txt"));
  } 

  // 一秒待つ
  delay(1000);
}

void dateTime(uint16_t* date, uint16_t* time)
{
  uint16_t year = 2022;
  uint8_t month = 6, day = 20, hour = 13, minute = 0, second = 0;

  // GPSやRTCから日付と時間を取得
  // FAT_DATEマクロでフィールドを埋めて日付を返す
  *date = FAT_DATE(year, month, day);

  // FAT_TIMEマクロでフィールドを埋めて時間を返す
  *time = FAT_TIME(hour, minute, second);
}
