#include <Stepper.h>
#include <ros.h>
#include <std_msgs/String.h>

// 1.8 deg/step
const int stepsPerRevolution = 200;

// Create Stepper instance
Stepper stepper(stepsPerRevolution, 8, 9, 10, 11);

// Callback, ros nodehandle, subscriber
void messageCallback( const std_msgs::String &msg) {
    digitalWrite(13, HIGH-digitalRead(13));
	stepper.step(stepsPerRevolution);
	delay(500);
	stepper.step(-stepsPerRevolution);
	delay(500);
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub("stepper_control", &messageCallback);

void setup() {

  // 60RPM
  stepper.setSpeed(60);

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
