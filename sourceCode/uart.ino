int Zone() //count zone
{ 
  int sensorValue = digitalRead(photoRes);
  if ((sensorValue == 1) && (isSent == false)) 
  {
    {
     isSent = true;
     zone = zone + 1;
    }
  }
  if (sensorValue == 0)  
    {
      isSent = false;
    }
  return zone;
 }
 
//UART from ESP
bool uart()
{
  bool Enable = 0;
  if (Serial2.available())
  {
    String readfrEsp = "";
    readfrEsp = Serial2.readString();
    Serial.println(readfrEsp);
    if (readfrEsp.compareTo("start") == 0)
    {
      Enable = 1;
    }
    if (readfrEsp.compareTo("stop") == 0)
    {
      Enable = 0;
    }
  }
  //Serial.println(Enable);
  return Enable;
}
