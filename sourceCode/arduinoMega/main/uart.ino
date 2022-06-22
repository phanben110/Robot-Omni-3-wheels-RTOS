//UART from ESP
int receive()
{
  String str;
  int Enable;
  if (Serial2.available())
  {
    str = Serial2.readStringUntil('\n');    // nhận ký tự
    int value = str.toInt();
    if (value == 1)
    {
      Enable = 1;
      if (DEBUG)
      {
        Serial.println("Start") ;
      }
    }
    else if (value == 0)
    {
      if (DEBUG)
      {
        Serial.println("Stop") ;
      }

      Enable = 0;
    }
    else
      Enable = -1;
  }
  return Enable;
}

//count zone
int Zone()
{
  int sensorValue = digitalRead(photoRes);
  //Serial.println(sensorValue);
  if ((sensorValue == 0) && (isSent == false))
  {
    {
      isSent = true;
      zone = zone + 1;
      Serial2.println(zone);
    }
  }
  if (sensorValue == 1)
  {
    isSent = false;
  }
  return zone;
}
//Check Left sensor
int LeftRight()
{
  int sensorNPN = digitalRead(checkNPN);
  if ( DEBUG )
  {
    Serial.print("checkNPN: ");
    Serial.println(digitalRead(checkNPN));
  }
  if ((sensorNPN == 0) && (isSent2 == false))
  {
    {
      isSent2 = true;
      countNPN = countNPN + 1;
    }
  }
  if (sensorNPN == 1)
  {
    isSent2 = false;
  }
  return countNPN ;
}
