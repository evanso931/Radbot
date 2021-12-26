// PART OF CODE TAKEN FROM JAMES BRUTON ROS ROBOT VIDEO

#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>


Adafruit_PCD8544 display = Adafruit_PCD8544(48, 46, 44);


////////////   PID and motor control varibales  and forwar declorations  ////////////

// right motor
double RightPk = 5.0; 
double RightIk = 20.0;
double RightDk = 0.00;

double RightSetpoint, RightInput, RightOutput, RightOutputA; // PID varibles 
PID PIDRight(&RightInput, &RightOutput, &RightSetpoint, RightPk, RightIk, RightDk, DIRECT); // PID varibles 

// left motor
double LeftPk = 5.0; 
double LeftIk = 20.0;
double LeftDk = 0.00;

double LeftSetpoint, LeftInput, LeftOutput, LeftOutputA; // PID varibles 
PID PIDLeft(&LeftInput, &LeftOutput, &LeftSetpoint, LeftPk, LeftIk, LeftDk, DIRECT); // PID varibles 

float RightDemand;
float LeftDemand;

float ForwardDemand;
float RotateDemand;

unsigned long CurrentMillis;
unsigned long PreviousMillis;
unsigned long onempersecond = 0;

//Left 
const byte LeftEncoderpinA = 2;//A pin -> the interrupt pin 0
const byte LeftEncoderpinB = 4;//B pin -> the digital pin 4

//right
const byte RightEncoderpinC = 3;//C pin -> the interrupt pin 1
const byte RightEncoderpinD = 5;//D pin -> the digital pin 5

byte LeftEncoderPinALast;
byte RightEncoderPinCLast;

volatile long LeftDuration= 0;//the number of the pulses left
volatile long RightDuration= 0;//the number of the pulses right

float LeftEncoderDiff = 0; //left
float RightEncoderDiff = 0; //right

float LeftEncoderError = 0; //left 
float RightEncoderError = 0; // Right

float LeftEncoderPrev = 0; //Left
float RightEncoderPrev = 0; //Right

boolean LeftDirection;//the rotation direction
boolean RightDirection;//the rotation direction

// Right Motor
int enA = 6;
int in1 = 7;
int in2 = 8;
 
// Left Motor
int enB = 11;
int in3 = 9;
int in4 = 10;

double speed_act_right = 0; 
double speed_act_left = 0;  


////////////   Gieger Counter Code from luke   ////////////

int micPin = A14;          //Input  Analogue Mic Pin
int relayPin = 12;        //int Relay Pin

int threshold = 60;       //0->1024 (difference value that is a detection)
int clicksRequired = 150; //required clicks for error
int clicksPerClick = 57;  //clicks added per detection
int clicksPerLoop = 2;    //clicks lost per loop

//Initial Values
int input = 200;        //Mic input value
int input_1 = 200;      //Last input value
int clicks = 0;         //Clicks

#define LOOPTIME  10

// Ros Subscriber Code 
ros::NodeHandle nh;

// Ros subsriber node from James Bruton, limits motor speed
void handle_cmd( const geometry_msgs::Twist& vel)
{
  ForwardDemand = vel.linear.x;
  RotateDemand = vel.angular.z;

  ForwardDemand = constrain(ForwardDemand, -0.25,0.25);
  RotateDemand = constrain(RotateDemand, -1,1);
}

// Ros publisher node from nox robot project 

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); 

void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}



void setup()
{
  // ROS setup 
  nh.initNode();                            //init ROS node
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  
  Serial.begin(57600);//Initialize the serial port
  
  EncoderInit();//Initialize the module
 
  // Set all the motor control pins to outputs
  TCCR4B = TCCR4B & B11111000 | B00000001;
  pinMode(enA, OUTPUT);
  TCCR1B = TCCR1B & B11111000 | B00000001; 
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  PIDRight.SetMode(AUTOMATIC);
  PIDRight.SetOutputLimits(-255,255);
  PIDRight.SetSampleTime(10);
  
  PIDLeft.SetMode(AUTOMATIC);
  PIDLeft.SetOutputLimits(-255,255);
  PIDLeft.SetSampleTime(10);


  /* Excel Graphing setup code using PLX_DAQ

  Serial.println("CLEARDATA"); //clears up any data left from previous projects

  Serial.println("LABEL,Date,Time,Milliseconds,EncoderCountsRight,EncoderCountsLeft"); 

  Serial.println("RESETTIMER"); //resets timer to 0

  Make robot move foward 1m when using excel code 
  demand1 = 0.25;  // right 
  demand2 = 0.25; // left

  */


  // Geiger setup 
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  unsigned int clicks = 0;         //Clicks


  // LCD setup 
  display.begin();
  display.setContrast(50);
  display.clearDisplay();  
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(8,10);
  display.println("Web Address");
  display.println("Time Remaining");
  display.println(CurrentMillis);
  display.display();
  

}

void EncoderInit()
{
  pinMode(LeftEncoderpinB,INPUT);
  pinMode(RightEncoderpinD,INPUT);
  
  attachInterrupt(0, LeftWheelSpeed, CHANGE);
  attachInterrupt(1, RightWheelSpeed, CHANGE);
}


////////////   Main program loop  ////////////

void loop() {
  CurrentMillis = millis();
  nh.spinOnce();
  
  // Time loop 
  if (CurrentMillis - PreviousMillis >= LOOPTIME) {
      PreviousMillis = CurrentMillis;  

      /*  Code for testing PID controller within arduno IDE and exporting datata to excel
      if (Serial.available()>0) {
          char c = Serial.read();
          if (c == 'w') {
             ForwardDemand = 0.25;  
             RotateDemand = 0;
          }
          else if (c == 'q'){
             ForwardDemand = 0;  
             RotateDemand = 0; 
          }
          else if (c == 's'){
             ForwardDemand = -0.25;   
             RotateDemand = 0; 
          }
          else if (c == 'd'){  
             ForwardDemand = 0;  
             RotateDemand = -1; 
          }
          else if (c == 'a'){
             ForwardDemand = 0;  
             RotateDemand = 1; 
          }
      }

      //Auduino IDE plotter code
      //Serial.println((String) Time + "," + LeftDuration + "," + RightDuration);
      //Serial.println((String) LeftEncoderDiff + "," + RightEncoderDiff + "," + RightSetpoint);
      //Serial.println((String) "DATA,DATE,TIME," + Time + "," + encoder0Diff + "," + encoder1Diff);

      */


      ////////////   Colculates motor speed from encoder counts, run PID control and outputs PWM values to motors   ////////////
      
      // distance between wheels is 28.5cm, half of that is 14.25cm = 142.5mm
      // Diff drive code
      RightDemand = ForwardDemand + (RotateDemand*0.1425);
      LeftDemand = ForwardDemand - (RotateDemand*0.1425);
      
      // Encoder counts per loop for 1m/s
      RightEncoderDiff = RightDuration - RightEncoderPrev;
      LeftEncoderDiff = LeftDuration - LeftEncoderPrev;

      speed_act_right =  RightEncoderDiff/164.6;
      speed_act_left = LeftEncoderDiff/166;
      
      //LeftEncoderError = (LeftDemand*166)- LeftEncoderDiff;
      //RightEncoderError = (RightDemand*165)- RightEncoderDiff;

      RightEncoderPrev = RightDuration;
      LeftEncoderPrev = LeftDuration;
      
      RightSetpoint = RightDemand*164.6;
      RightInput = RightEncoderDiff; //right

      LeftSetpoint = LeftDemand*166;
      LeftInput = LeftEncoderDiff;  //left

      CheckRadiation();
      
      PIDRight.Compute();
      PIDLeft.Compute();
     
 
      // H-bridge control
      
      if (RightOutput > 0) {
        RightOutputA = abs(RightOutput);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, RightOutputA); 
      }
      else if (RightOutput < 0) {
        RightOutputA = abs(RightOutput);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);   
        analogWrite(enA, RightOutputA);
      }
      else {
        analogWrite(enA, 0);
      }

      if (LeftOutput > 0) {
        LeftOutputA = abs(LeftOutput);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enB, LeftOutputA); 
      }
      else if (LeftOutput < 0) {
        LeftOutputA = abs(LeftOutput);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);  
        analogWrite(enB, LeftOutputA);
      }
      else {
        analogWrite(enB, 0);
      }

      publishSpeed(LOOPTIME);
  }
}


////////////   Gieger Counter Code from Luke with LCD control   ////////////

void CheckRadiation() {
  // put your main code here, to run repeatedly:
  input = analogRead(micPin);
  //Serial.println(input);
  if (input >=  threshold) {
    clicks = clicks + clicksPerClick;
   //Serial.println("CLICK");
  }
  if (clicks >= clicksRequired) {
    display.clearDisplay();
    display.setCursor(15,15);
    display.println("Radiation Found");
    display.display();
    digitalWrite(relayPin, LOW);
    for (int i=0; i<10; i++){
      i=0;
    }
     LeftSetpoint = 0;
     RightSetpoint = 0;
  }
  //delay(1);
  if (clicks>=1){
  clicks = clicks - clicksPerLoop;
  //Serial.println("ADD");
  }
}



////////////   ISR's for reading the encoders   ////////////

void LeftWheelSpeed()
{
  int Lstate = digitalRead(LeftEncoderpinA);
  if((LeftEncoderPinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(LeftEncoderpinB);
    if(val == LOW && LeftDirection)
    {
      LeftDirection = false; //Reverse
    }
    else if(val == HIGH && !LeftDirection)
    {
      LeftDirection = true;  //Forward
    }
  }
  LeftEncoderPinALast = Lstate;

  if(!LeftDirection)  {
    LeftDuration--;
  
  }else  {
    LeftDuration++;
  }
}


void RightWheelSpeed()
{
  int Lstate = digitalRead(RightEncoderpinC);
  if((RightEncoderPinCLast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(RightEncoderpinD);
    if(val == LOW && RightDirection)
    {
      RightDirection = false; //Reversed
    }
    else if(val == HIGH && !RightDirection)
    {
      RightDirection = true;  //Forward
    }
  }
  RightEncoderPinCLast = Lstate;

  if(!RightDirection)  {
    RightDuration++;
  
  }else  {
    RightDuration--;
  }
}
