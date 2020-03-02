// ----------------------------------------------------------
// NAME: Motor Control Sketch
// DESCRIPTION: Controls stepper motor for gantry positioning,
// and cutter servo to cut cucumber stems.
// ----------------------------------------------------------

#include <Servo.h>
#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

// Servo pin
#define servoPin 4

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Create servo instance
Servo cutterServo;
int servoPos = 0;

// Stepper callback function
void stepperMessageCallback( const std_msgs::Int32 &msg) {
  
  digitalWrite(13, HIGH);
  // Set the target position:
  stepper.moveTo(msg.data);
  // Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();
}

// Cutter callback function
void cutterMessageCallback( const std_msgs::Bool &msg) {

    for (servoPos = 0; servoPos <= 180; servoPos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        cutterServo.write(servoPos);        // tell servo to go to position in variable 'servoPos'
        delay(5);                       // waits 5ms for the servo to reach the position
    }
    for (servoPos = 180; servoPos >= 0; servoPos -= 1) { // goes from 180 degrees to 0 degrees
        cutterServo.write(servoPos);        // tell servo to go to position in variable 'servoPos'
        delay(5);                       // waits 5ms for the servo to reach the position
    } 
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> stepperSub("stepperControl", &stepperMessageCallback);
ros::Subscriber<std_msgs::Bool> cutterSub("cutterControl", &cutterMessageCallback);

void setup() {

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(6000);

  // Initialize LED pin
  pinMode(13, OUTPUT);

  // Initialize ROS nodes
  nh.initNode();
  nh.subscribe(stepperSub);
  nh.subscribe(cutterSub);
    
  // Attach servo
  cutterServo.attach(servoPin);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
