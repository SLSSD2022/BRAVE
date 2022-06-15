// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400
void setup() {
  Serial.begin(115200);

  Serial.print("Twelite Rover");
  Serial.println();
  pinMode(RST, OUTPUT);
  pinMode(BPS, OUTPUT);
  digitalWrite(BPS, LOW);
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);

  Serial2.begin(38400);

}

const int MaxBufferSize = 100;
char Buffer[MaxBufferSize];
void Parse();
int Buffer_pos = 0;
unsigned long message;
void  writeToTwelite (unsigned long message);

void loop() {
  if (Serial2.available() > 0){
      char c = Serial2.read();
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
        Serial2.print(":000100ABC123X/r/n");
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

void  writeToTwelite (unsigned long message){
  const int bufferLength = 8;
  unsigned char Buffer2[bufferLength];
  int i = 0;
  int j = 0;
  unsigned char temp = 0;
  unsigned char temp1,temp2;
  while(i<bufferLength){
         temp = ((message >> 4*i) & 0xFF); 
         //Serial.println(variable,HEX);
         temp1 = temp & 0b11110000;
         temp1 = temp1 >> 4;
         //Serial.println(temp1,HEX);
         temp2 = temp & 0b00001111;
         if (temp1<0xA){
             temp1 = temp1 + 0b00110000;
         }
         else{
             temp1 = temp1 + 0b00110111; 
         }
         if (temp2<0xA){
             temp2 = temp2 + 0b00110000;
         }
         else{
             temp2 = temp2 + 0b00110111; 
         }
         Buffer2[bufferLength-i-1] = temp2;
         Buffer2[bufferLength-i-2] = temp1;
         i=i+2;
         temp = 0;
  }
  while (j< bufferLength){
      Serial.print((char) Buffer2[j]);
      j++;
     }
}
