#ifndef _EEPROM_HEAD_
#define _EEPROM_HEAD_

//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
class EEPROM {
  public:
    uint8_t addrEEPROM = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
    unsigned int addrData = 30; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)
    void write(int addr_device, unsigned int addr_res, byte data );
    void writeInt(int addr_device, unsigned int addr_res, int data);
    void writeLong(int addr_device, unsigned int addr_res, unsigned long data);
    void writeFloat(int addr_device, unsigned int addr_res, float data);
    byte read(int addr_device, unsigned int addr_res );
    float readFloat(int addr_device, unsigned int addr_res);
    void Log();
    void LogGPSdata();
};

#endif
