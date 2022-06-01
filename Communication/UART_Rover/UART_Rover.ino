#include <SoftwareSerial.h>
SoftwareSerial MWSerial(2, 3);                   // RX, TX
void setup() {
  Serial.begin(115200);

  Serial.print("Twelite Rover");
  Serial.println();

  MWSerial.begin(38400);

}

const int MaxBufferSize = 100;
char Buffer[MaxBufferSize];
void Parse();
int Buffer_pos = 0;

void loop() {
  if (MWSerial.available() > 0){
      char c = MWSerial.read();
      if ( c != '\n' && (Buffer_pos < MaxBufferSize - 1) ){
        Buffer[Buffer_pos] = c;
        Buffer_pos++;
      }
      else
      {
        Buffer[Buffer_pos] = '\0';
        Serial.println(Buffer);
        Parse();
        Buffer_pos = 0;
        }
        
}
}

// TWELITE write its packets in HEX then converts it to characters. The Parse function retrieves the original HEX numbers as ints.
void Parse() {
  int i = 5;
  int d,e,f;
  int checker; 

  if(Buffer[3]=='8' && Buffer[4] =='1'){
    while (i< Buffer_pos-1){
      checker = Buffer[i] & 0b01000000;
      //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
      if (checker == 0b01000000){
        d = Buffer[i]+9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
      }
      else{
        d = Buffer[i];
      }
      d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
      d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
      //Same for the following character
      checker = Buffer[i+1] & 0b01000000;

      if (checker == 0b01000000){
        e = Buffer[i+1] +9;
      }
      else{
        e = Buffer[i+1];
      }
      e = e & 0b00001111;
      f = d+e;
      Serial.print("0x");
      Serial.println(f,HEX);
      i = i+2;
    }
  }
}  
 
