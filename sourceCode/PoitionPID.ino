#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include "PinChangeInt.h"
#include <PID_v1.h>                                   // Thanks to Brett Beauregard for his nice PID library
#include <Wire.h>
#include <mpu6050.h>
MPU6050 mpu6050;
const bool DEBUG = false ;
//motor intializations
const int inAPin1 = A4;
const int inBPin1 = A5;
const int PWMPin1 = 9;

const int inAPin2 = A2;
const int inBPin2 = A3;
const int PWMPin2 = 11;

const int inAPin3 = A0;
const int inBPin3 = A1;
const int PWMPin3 = 10;

const int SPEED = 80;

#define encoder1PinA  3    //motor 1
#define encoder1PinB  4
long unsigned int encoder1Pos = 1000;
#define encoder2PinA  2 //motor 2
#define encoder2PinB  5
long unsigned int encoder2Pos = 1000;
long unsigned int encoder3Pos = 1000;
#define encoder3PinA  18    //motor 3
#define encoder3PinB  6

//Sensor
#define checkLeft 25 // checkpoint (zone)
#define checkNPN 27 //check right
#define photoRes 23 // checkLeft

double kp = 5 , ki = 1 , kd = 0.01 ;  // modify kp, ki and kd for optimal performance
double input1 = 0, output1 = 0, setpoint1 = 0,  kp1 = 5 , ki1 = 1 , kd1 = 0.01 ;
double input2 = 0, output2 = 0, setpoint2 = 0,  kp2 = 5 , ki2 = 1 , kd2 = 0.01;
double input3 = 0, output3 = 0, setpoint3 = 0,  kp3 = 5 , ki3 = 1 , kd3 = 0.01;
double denta1 = 0, denta2 = 0, denta3 = 0;
float denta[3], rot;
//unsigned long T1=2000,T2,T3,T4,T5,t;
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
void TaskUART(void *pvParameters);
//void base1(float magnitude, float theta, int rot, float denta[3]);

int d1;                           // for direction of rotation of motors
int d2;
int d3;

int v1;
int v2;
int v3;
float vel_x;
float vel_y;
float theta;
int magnitude , countNPN ;
TaskHandle_t Task_Handle1;
TaskHandle_t Task_Handle2;

void setup() {
  Serial2.begin(115200);
  Serial.begin(115200);
  pinMode(encoder1PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder1PinB, INPUT_PULLUP);                  // quadrature encoder input B
  pinMode(encoder2PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(encoder3PinA, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encoder3PinB, INPUT_PULLUP);
  pinMode(checkLeft, INPUT);                // quadrature encoder input A
  pinMode(checkNPN, INPUT);
  pinMode(photoRes, INPUT);

  attachInterrupt(digitalPinToInterrupt(3), encoder1, FALLING);               // update encoder position
  attachInterrupt(digitalPinToInterrupt(2), encoder2, FALLING);
  attachInterrupt(digitalPinToInterrupt(18), encoder3, FALLING);
  //TCCR1B = TCCR1B & 0b11111000 | 1;                   // set 31KHz PWM to prevent motor noise
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;

  M2PID.SetMode(AUTOMATIC);
  M2PID.SetSampleTime(1);
  M2PID.SetOutputLimits(-255, 255);

  M1PID.SetMode(AUTOMATIC);
  M1PID.SetSampleTime(1);
  M1PID.SetOutputLimits(-255, 255);

  M3PID.SetMode(AUTOMATIC);
  M3PID.SetSampleTime(1);
  M3PID.SetOutputLimits(-255, 255);
  //mpu6050_begin();
  xTaskCreate(TaskLive, "Task1", 256, NULL, 2, &Task_Handle1);
  xTaskCreate(TaskUART, "Task2", 256, NULL, 1, &Task_Handle2);
  vTaskStartScheduler();
}

void loop() {}
void TaskLive(void *pvParameters)
{
  //  unsigned int Frequency = 0;
  //  TickType_t xLastWakeTime;
  //  xLastWakeTime = xTaskGetTickCount();
  //  Serial.println(t);
  //while (uart())
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
    zone = Zone(); // Đếm zone
    countNPN = LeftRight();
    Serial.println(countNPN);
    //int Enable = receive();
    int Enable = 1;
    if ( Enable == 0)
    {
      magnitude = 0 ;
      theta = 0;
      rot = 0 ;
      zone = 0;
    }
    //    else if ( Enable == 1 && zone < 3) {
    else if ( Enable == 1 && countNPN < 1) {
      /*
        Theta = -90 : Chạy thẳng,
        Theta = -180 :  Rẻ phải ( chạy ngang ) ,
        Theta = 0 : Rẻ trái,
      */
      magnitude = increase ;
      theta = -90;
      rot = 2   * current_dir; // Hệ số điều chỉnh
    }
    else if (Enable == 1 && countNPN >= 1 )
    {
      magnitude = 35;
      theta = 0;
      rot =   0.8 * current_dir;
    }
    else if (zone >= 5) {
      magnitude = 0 ;
      theta = 0;
      rot = 0;
      //    else if ((zone >= 3) && ( zone < 5   ))
      //    {
      //      magnitude = 35;
      //      theta = 180;
      //      rot =   0.8 * current_dir;
      //    }
      //    else if (zone >= 5) {
      //      magnitude = 0 ;
      //      theta = 0;
      //      rot = 0 ;
      //    }
      //    if ( DEBUG )
      //    {
      //      Serial.print("Zone : ") ;
      Serial.print(zone);
    }
    //      Serial.print(" -- Angle : ") ;
    //      Serial.print(current_dir) ;
    //      Serial.print(" --- denta1: ") ;
    //      Serial.print(denta[0]) ;
    //      Serial.print(" denta2: ") ;
    //      Serial.print(denta[1]) ;
    //      Serial.print(" denta3: ") ;
    //      Serial.println(denta[2]) ;
    //    }
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
void TaskUART(void *pvParameters)
{
 (void) pvParameters;
 unsigned int Frequency = 1;
 TickType_t xLastWakeTime;
 xLastWakeTime = xTaskGetTickCount();
 while (1)
 {
   Serial.println("Task2");
   uart();
   //Serial.println(Enable);
 }

 //vTaskDelayUntil(&xLastWakeTime, Frequency);
 vTaskDelay(100);
}
