#include <Arduino.h>
#include <SoftwareSerial.h>
#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = "FslaEJTx3tXexTyrobtWyQ7HfnUS4H5t";

//char ssid[] = "Nang Nhung";
//char pass[] = "0989122125";
//char ssid[] = "WIGIGI-2.4G";
//char pass[] = "cuocsongma";
char ssid[] = "HappiRoomPlus";
char pass[] = "Dat160420";
int pinValue;
int pinValue2 ;

WidgetLED led1(V3);
WidgetLED led2(V4);
WidgetLED led3(V5);
WidgetLED led4(V6);
WidgetLED led5(V7);

String zone = "0";
char msg[20] = {0};

const int tx = 4;
const int rx = 5;
SoftwareSerial mySerial(rx, tx);
BLYNK_WRITE(V1) // this command is listening when something is written to V1
{
  pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable


  //Serial.print("V1 button value is: "); // printing value to serial monitor
  //Serial.println(pinValue);
}

BLYNK_WRITE(V2) // this command is listening when something is written to V1
{
  pinValue2 = param.asInt(); // assigning incoming value from pin V1 to a variable


  //Serial.print("V1 button value is: "); // printing value to serial monitor
  //Serial.println(pinValue);
}

void setup()
{
  // Debug console
  Serial.begin(115200);
  mySerial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  led1.off();
  led2.off();
  led3.off();
  led4.off();
  led5.off();
  //  lcd.print(0, 0, "Zone=0");
}

void loop()
{
  Blynk.run();
  if (pinValue == 1 && pinValue2 == 0 )
  {
    Serial.println(1);  // xem
    Blynk.virtualWrite(V2, 0);

    //mySerial.print("start");  // lenh xuong arduino
  }
  else if (pinValue == 0 && pinValue2 == 1 )
  {
    Serial.println(2);  // xem
    Blynk.virtualWrite(V1, 0);
    //mySerial.print("start");  // lenh xuong arduino
  }

  else if (pinValue == 0 && pinValue2 == 0)
  {
    Serial.println(0);
    led1.off();
    led2.off();
    led3.off();
    led4.off();
    led5.off();
    //mySerial.print("stop");
  }


  // UART from arduino
  //  if (mySerial.available())
  //  {
  //    zone = mySerial.readString();
  //    sprintf(msg, "Zone=%s ", zone);
  //    Serial.println(msg);
  //    lcd.print(0, 0, msg);
  //  }
  if (Serial.available())
  {
    zone = Serial.readStringUntil('\n');
    int checkPoint = zone.toInt();
    //Serial.println(checkPoint);
    if (checkPoint == 1 )
    {
      led1.on();
      led2.off();
      led3.off();
      led4.off();
      led5.off();
    }
    else if ( checkPoint == 2 )
    {
      led1.on();
      led2.on();
      led3.off();
      led4.off();
      led5.off();
    }
    else if ( checkPoint == 3 )
    {
      led1.on();
      led2.on();
      led3.on();
      led4.off();
      led5.off();
    }
    else if ( checkPoint == 4 )
    {
      led1.on();
      led2.on();
      led3.on();
      led4.on();
      led5.off();
    }
    else if ( checkPoint == 5 )
    {
      led1.on();
      led2.on();
      led3.on();
      led4.on();
      led5.on();
    }
  }
  delay(100);
}
