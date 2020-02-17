#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Callback, ros nodehandle, subscriber
void messageCallback( const std_msgs::Int32 &msg) {
  
  digitalWrite(13, HIGH);
  // Set the target position:
  stepper.moveTo(msg.data);
  // Run to target position with set speed and acceleration/deceleration:
  stepper.runToPosition();
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> sub("stepperControl", &messageCallback);

void setup() {

  // Set the maximum speed and acceleration:
  stepper.setMaxSpeed(10000);
  stepper.setAcceleration(6000);

  // Initialize LED pin
  pinMode(13, OUTPUT);

  // Initialize ROS nodes
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
