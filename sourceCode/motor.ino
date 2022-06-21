void M1(int out) {                                // to H-Bridge board
//  Serial.print("PWM: ");
//  Serial.println(out);
  if (out > 0) {
    digitalWrite(inAPin1, LOW);
    digitalWrite(inBPin1, HIGH);// drive motor CW
    analogWrite(PWMPin1, out);
  }
  else {
    digitalWrite(inAPin1, HIGH);
    digitalWrite(inBPin1, LOW);// drive motor CW
    analogWrite(PWMPin1, abs( out));                       // drive motor CCW
  }
}

void M2(int out) {                                // to H-Bridge board
  //  Serial.print("PWM: ");
  //  Serial.println(out);
  if (out > 0) {
    digitalWrite(inAPin2, HIGH);
    digitalWrite(inBPin2, LOW);// drive motor CW
    analogWrite(PWMPin2, out);
  }
  else {
    digitalWrite(inAPin2, LOW);
    digitalWrite(inBPin2, HIGH);// drive motor CW
    analogWrite(PWMPin2, abs( out));                       // drive motor CCW
  }
}
//
void M3(int out) {                                // to H-Bridge board
  //  Serial.print("PWM: ");
  //  Serial.println(out);
  if (out > 0) {
    digitalWrite(inAPin3, HIGH);
    digitalWrite(inBPin3, LOW);// drive motor CW
    analogWrite(PWMPin3, out);
  }
  else {
    digitalWrite(inAPin3, LOW);
    digitalWrite(inBPin3, HIGH);// drive motor CW
    analogWrite(PWMPin3, abs( out));                       // drive motor CCW
  }
}
