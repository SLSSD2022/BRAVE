#include <Tinygps++.h>
#include "./GPS.h"

//=========GPS and position function============================================================================//

// void GPSupdateGPSlocation() {
//   while (Serial1.available() > 0)
//   {
//     //    Serial.print("YES");
//     char c = Serial1.read();
//     //    Serial.print(c);
//     gps.encode(c);
//     if (gps.location.isUpdated())
//     {
//       Serial.println("");
//       Serial.println("I got new GPS!");
//       latR = gps.location.lat();  // roverの緯度を計算
//       lngR = gps.location.lng(); // roverの経度を計算
//       break;
//     }
//     //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
//     delay(10);
//   }
// }
