void encoder1()  {                                     // pulse and direction, direct port reading to save cycles
  //  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  //  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
  if (digitalRead(encoder1PinB) == HIGH)   encoderPos1++;
  if (digitalRead(encoder1PinB) == LOW)   encoderPos1--;
}
void encoder2()  {                                     // pulse and direction, direct port reading to save cycles
  //  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  //  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
  if (digitalRead(encoder2PinB) == HIGH)   encoderPos2++;
  if (digitalRead(encoder2PinB) == LOW)   encoderPos2--;
}

void encoder3()  {                                     // pulse and direction, direct port reading to save cycles
  //  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  //  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
  if (digitalRead(encoder3PinB) == HIGH)   encoderPos3++;
  if (digitalRead(encoder3PinB) == LOW)   encoderPos3--;

}
