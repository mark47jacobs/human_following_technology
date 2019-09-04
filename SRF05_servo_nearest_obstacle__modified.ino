/*
SRF05_servo_nearest_obstacle - Find the nearest obstacle and turn the sensor in its direction 
Connections:Vcc - 7, TRIG - 6 ,ECHO - 5 ,Gnd - 4 

Servo -> RED - 5V, BROWN - GND ,ORANGE - 9
*/
#include <Servo.h> 

//ultrasonic sensor pins
#define TRIG A1 // the SRF05 Trig pin
#define ECHO A2 // the SRF05 Echo pin
#define SENSOR_STEP 5 // 5 degrees

//L298N control pins
const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int RightMotorForward = 4;
const int RightMotorBackward = 5;

int numberOfSteps = int(180 / SENSOR_STEP);
unsigned long pulseTime;
unsigned long srfDistanceArray[180 / SENSOR_STEP]; // 180/step (where step is equal to 5)
unsigned long minimumDistanceThreshold = 5;
unsigned long maximumDistanceThreshold = 100;

Servo servo1;  // servo control object

void setup()
{
  pinMode(TRIG,OUTPUT);
  pinMode(ECHO,INPUT);
  servo1.attach(10);
  
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
   
}

float measurement()
{
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  // wait for the pulse to return. The pulse
  // goes from low to HIGH to low, so we specify
  // that we want a HIGH-going pulse below:
  pulseTime = pulseIn(ECHO,HIGH);
  int distance = (pulseTime*0.0343)/2;
  return distance;
}
boolean checkNeighborhood(int arrayIndex)
{
  unsigned long loweNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex-1]);
  unsigned long upperNeighborDiff = abs(srfDistanceArray[arrayIndex]-srfDistanceArray[arrayIndex+1]);
  if (loweNeighborDiff<20 || upperNeighborDiff<20)
  {
    return true;
  }
  else{
    return false;
  }
}
int measurementsDataMin()
{
  unsigned long minDistance = maximumDistanceThreshold; // srfDistanceArray[0];
  int index = 0;
  for (int i=1; i<numberOfSteps-2; i++){
    if(srfDistanceArray[i]>minimumDistanceThreshold && srfDistanceArray[i] < minDistance){
      if(checkNeighborhood(i)){ // if TRUE
        index = i;
        minDistance = srfDistanceArray[i];
      }
    }  
  }
  index = index*SENSOR_STEP;
  return index;
}
float timeVal(int angle)
{     float t = (5.93*angle);
      return t;
}

void loop()
{
  int pos;
  int nearestObjectPosition;
  int arrayIndex = 0;
  servo1.write(0);     // Tell servo to go to 0 degrees
  delay(1000);         // Pause to get it time to move

  for(pos = 0; pos < 180; pos += SENSOR_STEP) // Tell servo to go to 180 degrees, stepping by 5 degrees
{
    servo1.write(pos);  // Move to next position
    delay(30);               // Short pause to allow it to move, min 20ms
    if(arrayIndex<numberOfSteps-1)
    {
          srfDistanceArray[arrayIndex] = measurement(); 
    }
    arrayIndex++;
}
  
  nearestObjectPosition = measurementsDataMin();
  servo1.write(nearestObjectPosition);
  delay(7000);         
}
