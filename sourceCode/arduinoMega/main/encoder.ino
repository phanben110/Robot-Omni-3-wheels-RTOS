
// pulse and direction, direct port reading to save cycles
void encoder1()
{
  if (digitalRead(encoder1PinB) == HIGH)   encoderPos1++;
  if (digitalRead(encoder1PinB) == LOW)   encoderPos1--;
  if ( DEBUG )
  {
    Serial.print("Encoder1");
    Serial.println(encoderPos1);
  }

}
// pulse and direction, direct port reading to save cycles
void encoder2()  {
  if (digitalRead(encoder2PinB) == HIGH)   encoderPos2++;
  if (digitalRead(encoder2PinB) == LOW)   encoderPos2--;
  if ( DEBUG )
  {
    Serial.print("Encoder2");
    Serial.println(encoderPos2);
  }
}
// pulse and direction, direct port reading to save cycles
void encoder3()  {

  if (digitalRead(encoder3PinB) == HIGH)   encoderPos3++;
  if (digitalRead(encoder3PinB) == LOW)   encoderPos3--;
  if ( DEBUG )
  {
    Serial.print("Encoder3");
    Serial.println(encoderPos3);
  }

}
