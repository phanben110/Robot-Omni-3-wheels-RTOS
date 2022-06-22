#include "definePin.h"
const bool debugPWM = false ;
// to H-Bridge board
void M1(int out)
{
  if ( debugPWM )
  {
    Serial.print("PWM M1: ");
    Serial.println(out);
  }
  // drive motor CW
  if (out > 0) {
    digitalWrite(inAPin1, LOW);
    digitalWrite(inBPin1, HIGH);
    analogWrite(PWMPin1, out);
  }
  // drive motor CCW
  else {
    digitalWrite(inAPin1, HIGH);
    digitalWrite(inBPin1, LOW);
    analogWrite(PWMPin1, abs( out));
  }
}

void M2(int out)
{
  if ( debugPWM )
  {
    Serial.print("PWM M2: ");
    Serial.println(out);
  }
  // drive motor CW
  if (out > 0) {
    digitalWrite(inAPin2, HIGH);
    digitalWrite(inBPin2, LOW);
    analogWrite(PWMPin2, out);
  }
  // drive motor CCW
  else {
    digitalWrite(inAPin2, LOW);
    digitalWrite(inBPin2, HIGH);
    analogWrite(PWMPin2, abs( out));
  }
}

void M3(int out)
{
  if ( debugPWM )
  {
    Serial.print("PWM M3: ");
    Serial.println(out);
  }
  // drive motor CW
  if (out > 0) {
    digitalWrite(inAPin3, HIGH);
    digitalWrite(inBPin3, LOW);
    analogWrite(PWMPin3, out);
  }
  // drive motor CW
  else {
    digitalWrite(inAPin3, LOW);
    digitalWrite(inBPin3, HIGH);
    analogWrite(PWMPin3, abs( out));
  }
}
