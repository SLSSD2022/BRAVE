#ifndef _EEPROM_HEAD_
#define _EEPROM_HEAD_

//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
class EEPROM {
public:
  uint8_t addrEEPROM;
  uint8_t addrData;

  EEPROM();
  EEPROM(uint8_t addr);
  void init(uint8_t addr);
  void write(uint8_t addr_res, byte data );
  void writeInt(uint8_t addr_res, int data);
  void writeLong(uint8_t addr_res, unsigned long data);
  void writeFloat(uint8_t addr_res, float data);
  byte read(uint8_t addr_res );
  float readFloat(uint8_t addr_res);
  //void log();
  //void logGPSdata();
};

#endif
