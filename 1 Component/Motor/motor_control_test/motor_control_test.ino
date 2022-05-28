const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;

float degRtoA = 90;//目的方向(仮)
float x =0;//機軸方向
float delta_theta; //目的方向(仮)と機軸方向の差分
float threshold = 30; //角度の差分の閾値


int Normal_speed = 100;
int speed_R;
int speed_L;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENABLE,OUTPUT);  // 7番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 10番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 11版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 12番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 13版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
  delay(500);
}

void loop() {
  Serial.print("axis:");
  Serial.print(x);
  Serial.print(":");
  

  if (x < degRtoA){
      delta_theta = degRtoA - x;
      Serial.print("x < degRtoA:");
      Serial.print(delta_theta);
      Serial.print(":");
      
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= degRtoA)){
        speed_R = Normal_speed;
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print("Go straight");
      }
      //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Normal_speed;
        speed_L = Normal_speed - (delta_theta * Normal_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L ); 
        Serial.print("turn left");
      }
  
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else { 
        speed_R = Normal_speed - ((360-delta_theta) * Normal_speed / 180);
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print("turn right");
      }
  }

  
  else {
      delta_theta = x - degRtoA;
      Serial.print("degRtoA < x:");
      Serial.print(delta_theta);
      Serial.print(":");
     
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= 360)){
        speed_R = Normal_speed;
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print("Go straight");
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Normal_speed - (delta_theta * Normal_speed / 180);
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print("turn right");
      }
  
      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else { 
        speed_R = Normal_speed;
        speed_L = Normal_speed - ((360-delta_theta) * Normal_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print("turn left");
        
      }
  }
  Serial.print(",");
  Serial.print(speed_L);
  Serial.print(",");
  Serial.println(speed_R);

  x += 5;
  if(x > 360){
    x -= 360;
  }
  delay(1000);
}
