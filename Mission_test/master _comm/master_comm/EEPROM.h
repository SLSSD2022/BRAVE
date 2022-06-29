//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
uint8_t addrEEPROM = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
unsigned int addrData = 30; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)

void writeEEPROM(int addr_device, unsigned int addr_res, byte data );
void EEPROM_write_int(int addr_device, unsigned int addr_res, int data);
void EEPROM_write_long(int addr_device, unsigned int addr_res, unsigned long data);
void EEPROM_write_float(int addr_device, unsigned int addr_res, float data);
byte readEEPROM(int addr_device, unsigned int addr_res );
float EEPROM_read_float(int addr_device, unsigned int addr_res);
void LogToEEPROM();
void LogGPSdata();
