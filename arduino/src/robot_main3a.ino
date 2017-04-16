#if (ARDUINO >=100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <ros/time.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <math.h>

#define  MOTOR_RATE 10 //hz
unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;

//header file for stepper motor
#include <AccelStepper.h>
#define stp 22
#define dir 23
#define MS1 19
#define MS2 20
#define MS3 21
#define EN 18

//stepper object
AccelStepper stepper(4,22,23,19,20);

//store direction and required steps
geometry_msgs::Pose moveArray;

//messages
//std_msgs::UInt16 stageMsg;
//std_msgs::UInt16 stepMsg;

//stage steps definition
int stage1 = 0;
int stage2 = 200;
int stage3 = 400;

//Servo header
#include <Servo.h>
Servo myservo;
#define servo_pin 9

//Switch
#define s1 14
#define s2 15

//ultrasonic
#include <NewPing.h>
#define TRIGGER_PIN  12
#define ECHO_PIN     11 
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 
//sonar.ping_cm()

//other variables
int desired_stage; //MotorCb
int current_step = 0;
int current_stage = 0;
int direction = 0;
int reqStepTo = 0;
int turnDir;

//function prototypes
void checkStage(int Stage);
void currentStageis(long StepVal);
void MotorCb(const std_msgs::UInt16& motor_msg);
void servoCb(const std_msgs::UInt16& servo_msg);
void ultrasonic();

ros::NodeHandle nh;

ros::Publisher basePub("base", &moveArray);
ros::Subscriber<std_msgs::UInt16> motorSub("cmdmotor", MotorCb);
//ros::Subscriber<std_msgs::Int16> servoSub("cmdservo", servoCb);
ros::Subscriber<std_msgs::UInt16> servoSub("cmdservo", servoCb);

void setup()
{
  nh.initNode();

  pinMode(13, OUTPUT);
  myservo.attach(servo_pin);

  pinMode(stp, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);


  digitalWrite(stp, LOW);
  digitalWrite(dir, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
  
 
  nh.subscribe(motorSub);
  nh.subscribe(servoSub);
  nh.advertise(basePub);


  digitalWrite(EN, LOW);

  //switch
  pinMode(s1, INPUT);
}


void loop()
{

  if((millis() - g_prev_command_time) >= (1000/ MOTOR_RATE))
  {
      digitalWrite(13, HIGH);

      ultrasonic();
	
      //switch check
      if(digitalRead(s1) == 1)
      {
            moveArray.orientation.y = 1;      	
      }
      if(digitalRead(s1) == 0)
      {
            moveArray.orientation.y = 0;
      }

      g_prev_control_time = millis();

  }

  basePub.publish(&moveArray);
  nh.spinOnce();
  delay(5);
}


//callback function prototypes
void MotorCb( const std_msgs::UInt16& motor_msg)
{
    	digitalWrite(13, LOW);

    	desired_stage = motor_msg.data;
   	checkStage(desired_stage);


   	if(moveArray.position.x == 0)
   	{
     		digitalWrite(dir, 0);
   	}
   	if(moveArray.position.x == 1)
   	{
   	  	digitalWrite(dir,1);
   	}
	//int reqStepTo = moveArray.position.y;
	moveArray.position.y = reqStepTo;
   	for(int x = 0; x < reqStepTo; x ++)
   	{
      		digitalWrite(stp, 1);
      		delay(1);
      		digitalWrite(stp, 0);
      		delay(1);
		
		moveArray.position.y--;
    	}
}

//callback function servo
void servoCb(const std_msgs::UInt16& servo_msg)
{
	moveArray.orientation.x = servo_msg.data;
	if(moveArray.orientation.x == 1)
	{
		turnDir = 85;
	}	
	else
	{	
		turnDir = 50;
	}
	myservo.write(servo_msg.data);
}



void checkStage(int Stage)
{
//change it to pass by reference
	if (Stage < current_stage)
	{
		direction = 0;
	}
	else
	{
		direction = 1;
	}

	int diff = abs(Stage - current_stage);
	current_stage = Stage;
	
	
	moveArray.position.x=direction;
	reqStepTo = diff*6.667;
	//moveArray.position.y=diff*6.667;
	moveArray.position.z=current_stage*6.667;
}


void ultrasonic()
{
	moveArray.orientation.z = sonar.ping_cm();
}
