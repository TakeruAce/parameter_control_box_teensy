#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

int slide[4] = {0,0,0,0};
int slidePin[4] = {14,15,16,17};

// ros interface
ros::NodeHandle nh;

std_msgs::Float64MultiArray msg_inertia_diff;
std_msgs::Float64MultiArray msg_viscosity_diff;
std_msgs::Float64MultiArray msg_wheel;
std_msgs::Bool msg_activate;
ros::Publisher pub_inertia_diff("/torque_controller/impedance/inertia_diff_command", &msg_inertia_diff);
ros::Publisher pub_viscosity_diff("/torque_controller/impedance/viscosity_diff_command", &msg_viscosity_diff);
ros::Publisher pub_wheel("/torque_controller/wheels_command", &msg_wheel);
ros::Publisher pub_activate("/torque_controller/enable_motion_combination", &msg_activate);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(2,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(3,OUTPUT);

  msg_inertia_diff.data = (float*) malloc(sizeof(float) * 3);
  msg_inertia_diff.data_length = 3;
  msg_viscosity_diff.data = (float*) malloc(sizeof(float) * 3);
  msg_viscosity_diff.data_length = 3;
  msg_wheel.data = (float*) malloc(sizeof(float) * 1);
  msg_wheel.data_length = 1;
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pub_inertia_diff);
  nh.advertise(pub_viscosity_diff);
  nh.advertise(pub_wheel);
  nh.advertise(pub_activate);
}

void loop() {
  // put your main code here, to run repeatedly
  for(int i=0;i<sizeof(slide)/sizeof(slide[0]);i++) {
    slide[i] = analogRead(slidePin[i]);
  }
  int button1 = digitalRead(2);
  int button2 = digitalRead(4);
  digitalWrite(3, !button1);

  Serial.print(slide[0]);
  Serial.print(",");
  Serial.print(slide[1]);
  Serial.print(",");
  Serial.print(slide[2]);
  Serial.print(",");
  Serial.print(slide[3]);
  Serial.print(",");
  Serial.print(button1);
  Serial.print(",");
  Serial.print(button2);
  Serial.print(",");
  Serial.println();

  msg_inertia_diff.data[0] = slide[0];
  msg_inertia_diff.data[1] = slide[0];
  msg_inertia_diff.data[2] = slide[0];

  msg_viscosity_diff.data[0] = slide[1];
  msg_viscosity_diff.data[1] = slide[1];
  msg_viscosity_diff.data[2] = slide[1];
  if (!button1) {
    msg_wheel.data[0] = 800;
  } else {
    msg_wheel.data[0] = 0;
  }

  if (!button2) {
    msg_activate.data = true;
  } else {
    msg_activate.data = false;
  }

  pub_inertia_diff.publish(&msg_inertia_diff);
  pub_viscosity_diff.publish(&msg_viscosity_diff);
  pub_wheel.publish(&msg_wheel);
  pub_activate.publish(&msg_activate);

  delay(100);
  nh.spinOnce();
}