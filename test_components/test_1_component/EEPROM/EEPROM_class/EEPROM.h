#ifndef _EEPROM_HEAD_
#define _EEPROM_HEAD_

//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
class EEPROM {
  public:
    uint8_t addrEEPROM;
    unsigned int addrData;
    void init();
    void setAddr(uint8_t addr);
    void write(unsigned int addr_res, byte data );
    void writeInt(unsigned int addr_res, int data);
    void writeLong(unsigned int addr_res, unsigned long data);
    void writeFloat(unsigned int addr_res, float data);
    byte read(int addr_device, unsigned int addr_res );
    float readFloat(int addr_device, unsigned int addr_res);
    //void log();
    //void logGPSdata();
};

#endif
