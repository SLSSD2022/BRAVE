const int anPin = 0;

long anVolt, cm;



void setup() {

   Serial.begin(9600);

}



void read_sensor(){

   anVolt = analogRead(anPin);

   cm = anVolt/2;

}



void print_range(){

   Serial.print("Range=");

   Serial.print(cm);

   Serial.print("cm");

   Serial.print('\n');

}

 
void loop() { 

   read_sensor();

   print_range();

   delay(100);

}
