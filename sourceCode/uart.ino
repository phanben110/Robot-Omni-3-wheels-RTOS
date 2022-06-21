int Zone() //count zone
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
int LeftRight() //count zone
{                                                                                                                                                                                                                                                                                                                                               
  int sensorNPN = digitalRead(checkNPN);
  Serial.println(digitalRead(checkNPN));
  //Serial.println(sensorValue);
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
//UART from ESP
int receive() { // Nhận một chuỗi ký tự từ ESP8266 rồi xử lý ---- vd: box1 0 05 10 20 (viết liền) thì: 0-trạng thái đóng mở, 05- sáng uống 0.5 viên, ...
  String str;
  int Enable;
  if (Serial2.available()) {
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
