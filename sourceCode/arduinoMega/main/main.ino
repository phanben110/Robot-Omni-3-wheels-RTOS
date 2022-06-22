#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include "PinChangeInt.h"
#include "PID_v1.h"
#include "mpu6050.h"
#include "motor.h"
#include "vectoring.h"
#include "definePin.h"

MPU6050 mpu6050;
const bool DEBUG = false ;
const int SPEED = 80;
double kp = 5 , ki = 1 , kd = 0.01 ;  // modify kp, ki and kd for optimal performance
double input1 = 0, output1 = 0, setpoint1 = 0,  kp1 = 5 , ki1 = 1 , kd1 = 0.01 ;
double input2 = 0, output2 = 0, setpoint2 = 0,  kp2 = 5 , ki2 = 1 , kd2 = 0.01;
double input3 = 0, output3 = 0, setpoint3 = 0,  kp3 = 5 , ki3 = 1 , kd3 = 0.01;
double denta1 = 0, denta2 = 0, denta3 = 0;
float denta[3], rot;
long temp;
bool Enable = 0, isSent = 0, isSent2 = 0;
int zone = 0 ;

volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
volatile long encoderPos3 = 0;

PID M1PID(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  // if motor will only run at full speed try 'REVERSE' instead of 'DIRECT'
PID M2PID(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);
PID M3PID(&input3, &output3, &setpoint3, kp3, ki3, kd3, DIRECT);

void TaskLive( void *pvParameters );
//void TaskUART(void *pvParameters);

int v1;
int v2;
int v3;
float vel_x;
float vel_y;
float theta;
int magnitude , countNPN ;
TaskHandle_t Task_Handle1;
//TaskHandle_t Task_Handle2;

void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
  pinMode(encoder1PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder1PinB, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(encoder2PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder2PinB, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(encoder3PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder3PinB, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(checkLeft, INPUT);
  pinMode(checkNPN, INPUT);
  pinMode(photoRes, INPUT);
  // update encoder position
  attachInterrupt(digitalPinToInterrupt(3), encoder1, FALLING);
  attachInterrupt(digitalPinToInterrupt(2), encoder2, FALLING);
  attachInterrupt(digitalPinToInterrupt(18), encoder3, FALLING);
  // set timer 31KHz to read encoder
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  //PID M1 intializations
  M1PID.SetMode(AUTOMATIC);
  M1PID.SetSampleTime(1);
  M1PID.SetOutputLimits(-255, 255);

  //PID M1 intializations
  M2PID.SetMode(AUTOMATIC);
  M2PID.SetSampleTime(1);
  M2PID.SetOutputLimits(-255, 255);

  //PID M1 intializations
  M3PID.SetMode(AUTOMATIC);
  M3PID.SetSampleTime(1);
  M3PID.SetOutputLimits(-255, 255);

  //create task in RTOS
  xTaskCreate(TaskLive, "Task1", 256, NULL, 2, &Task_Handle1);
  //xTaskCreate(TaskUART, "Task2", 256, NULL, 1, &Task_Handle2);
  vTaskStartScheduler();
}

void loop() {}
void TaskLive(void *pvParameters)
{
  // Code test run robot Omni 3 wheels
  mpu6050_begin();
  float increase = 30;
  float ROT = 3;
  float a = 0.5;
  while (1)
  {
    if (increase <= SPEED ) // tăng tốc dần lên increase
    {
      increase += 0.25 ;
    }
    float current_dir = mpu6050_yaw();
    base1(magnitude, theta, rot, denta);
    countNPN = LeftRight();
    Serial.println(countNPN);
    // Receive signal control from UART with ESP8266
    int Enable = receive();
    if ( Enable == 0)
    {
      magnitude = 0 ;
      theta = 0;
      rot = 0 ;
      zone = 0;
    }
    else if ( Enable == 1 && countNPN < 1)    // Go Straight
    {
      /*
        Theta = -90 : Chạy thẳng,
        Theta = -180 :  Rẻ phải ( chạy ngang ) ,
        Theta = 0 : Rẻ trái,
      */
      magnitude = increase ;
      theta = -90;
      rot = 2   * current_dir; // Hệ số điều chỉnh
    }
    else if (Enable == 1 && countNPN == 1 )   //Turn Right
    {
      magnitude = 35;
      theta = 0;
      rot =   0.8 * current_dir;
      Serial2.println(3);
    }
    else if (Enable == 2 && countNPN == 1 )  //Turn Left
    {
      magnitude = 35;
      theta = -180;
      rot =   0.8 * current_dir;
      Serial2.println(3);
    }
    else if (countNPN == 2)   //Stop
    {
      magnitude = 0 ;
      theta = 0;
      rot = 0;
      Serial2.println(5);
    }
    if ( DEBUG )
    {
      Serial.print(" -- Angle : ") ;
      Serial.print(current_dir) ;
      Serial.print(" --- denta1: ") ;
      Serial.print(denta[0]) ;
      Serial.print(" denta2: ") ;
      Serial.print(denta[1]) ;
      Serial.print(" denta3: ") ;
      Serial.println(denta[2]) ;
    }

    setpoint1 += denta[0];
    setpoint2 += denta[1];
    setpoint3 += denta[2];

    input1 = encoderPos1 ;
    input2 = encoderPos2 ;
    input3 = encoderPos3 ;

    M1PID.Compute();
    M2PID.Compute();
    M3PID.Compute();

    M1(output1);
    M2(output2);
    M3(output3);
    vTaskDelay(1);
  }
}
//void TaskUART(void *pvParameters)
//{
//  (void) pvParameters;
//  unsigned int Frequency = 1;
//  TickType_t xLastWakeTime;
//  xLastWakeTime = xTaskGetTickCount();
//  while (1)
//  {
//    Serial.println("Task2");
//    receive();
//  }
//
//  //vTaskDelayUntil(&xLastWakeTime, Frequency);
//  vTaskDelay(100);
//}
