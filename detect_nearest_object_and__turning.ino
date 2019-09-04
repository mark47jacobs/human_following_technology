/*
SRF05_servo_nearest_obstacle - Find the nearest obstacle and turn the sensor in its direction then turn the robot in that direction
Connections:Vcc - 7, TRIG - 6 ,ECHO - 5 ,Gnd - 4 

Servo -> RED - 5V, BROWN - GND ,ORANGE - 9
*/
#include <Servo.h> 

//ultrasonic sensor pins
#define TRIG A1 // the SRF05 Trig pin
#define ECHO A2 // the SRF05 Echo pin

#define SENSOR_STEP 4 // 4 degrees

//L298N control pins
const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int RightMotorForward = 4;
const int RightMotorBackward = 5;

int numberOfSteps = int(180 / SENSOR_STEP);
unsigned long pulseTime;
unsigned long srfDistanceArray[180 / SENSOR_STEP]; // 180/step (where step is equal to 4)
unsigned long minimumDistanceThreshold = 5;
unsigned long maximumDistanceThreshold = 100;

float time_of_turning = 0;
int indexWithMinDist=0;

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
  for (int i=1; i<numberOfSteps-2; i++)
  {
    if(srfDistanceArray[i]>minimumDistanceThreshold && srfDistanceArray[i] < minDistance)
    {
      if(checkNeighborhood(i))
      { 
        indexWithMinDist = i;
        minDistance = srfDistanceArray[i];
      }
    }  
  }
  index = indexWithMinDist*SENSOR_STEP;
  return index;
}
float timeVal(int angle)
{     float t = (1.96*angle);
      return t;
}
void turnRight()
{
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  
  delay(time_of_turning);
  
  digitalWrite(LeftMotorForward, LOW);

  digitalWrite(RightMotorBackward, LOW);
}

void turnLeft()
{
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(time_of_turning);
  
  digitalWrite(RightMotorForward, LOW);
  
  digitalWrite(LeftMotorBackward, LOW);
}

void setRobotPosition()
{   //detecting the nearest object   
    int pos;
    int nearestObjectPosition;
    int arrayIndex = 0;
    servo1.write(0);     // Tell servo to go to 0 degrees
    delay(1000);         // Pause to get it time to move
  
    for(pos = 0; pos < 180; pos += SENSOR_STEP) // Tell servo to go to 180 degrees, stepping by 5 degrees
    {
      servo1.write(pos);  // Move to next position
      delay(24);          // Short pause to allow it to move, min 20ms
      if(arrayIndex<numberOfSteps-1)
      {
            srfDistanceArray[arrayIndex] = measurement(); 
      }
      arrayIndex++;
    }
    
    nearestObjectPosition = measurementsDataMin();
    servo1.write(nearestObjectPosition);
    delay(4000);

    //turning the robot

    float angle = 90 -(nearestObjectPosition<=90 ? nearestObjectPosition : 180-nearestObjectPosition) ;

    time_of_turning = timeVal(angle); 
    if(nearestObjectPosition<=90)
    {       turnRight();        }
    else
    {       turnLeft();         }

    servo1.write(90);
    float newDist = measurement();
    float oldMin = srfDistanceArray[indexWithMinDist];
    
    delay(500);
    if ((newDist<oldMin+2.5)&&(newDist>oldMin-2.5))  
    {     delay(7000);       }
    else
    {     setRobotPosition();}
}

void loop()
{
      setRobotPosition()    ;/*note in the final code this function will be called by setup function*/
}
