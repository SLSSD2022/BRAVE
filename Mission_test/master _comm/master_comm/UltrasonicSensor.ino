//=========Ultrasonic sensor function============================================================================//
unsigned int getUltrasonic_HEAD() {
  long duration;
  digitalWrite(HEAD_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(HEAD_Trig, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(HEAD_Echo, HIGH);
  return microsecTocm(duration);
}

unsigned int microsecTocm(long microsec) {
  return (unsigned int) microsec / 29 / 2;
}