#ifndef _EXPROM_HEAD_
#define _EXPROM_HEAD_

//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
class EXPROM {
public:
  uint8_t addrEXPROM;
  uint8_t addrData;

  EXPROM();
  EXPROM(uint8_t addr);
  void init(uint8_t addr);
  void write(uint8_t addr_res, byte data );
  void writeInt(uint8_t addr_res, int data);
  void writeLong(uint8_t addr_res, unsigned long data);
  void writeFloat(uint8_t addr_res, float data);
  byte read(uint8_t addr_res );
  float readFloat(uint8_t addr_res);
  void log();
  void logGPSdata();
};

#endif
