/*
 Copyright (c) 2016, Juan Jimeno

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of  nor the names of its contributors may be used to
 endorse or promote products derived from this software without specific
 prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 */
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros.h>

//header file for publishing velocities for odom
#include <lino_msgs/Velocities.h>

//header file for cmd_subscribing to "cmd_vel"
#include <geometry_msgs/Twist.h>

//header file for pid server
#include <lino_msgs/PID.h>

//header files for imu
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include <ros/time.h>

#include <Wire.h>

#include "lino_base_config.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"

#define COMMAND_RATE 10 //hz


#ifdef L298_DRIVER
  //left side motors
  Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); //front
  //right side motors
  Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front
#endif

#ifdef BTS7960_DRIVER
  //left side motors
  Motor motor1(MOTOR1_IN_A, MOTOR1_IN_B); // front
  // right side motors
  Motor motor2(MOTOR2_IN_A, MOTOR2_IN_B); // front
#endif

//COUNTS_PER_REV = 0 if no encoder
int Motor::counts_per_rev_ = 0.0;

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);

double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;

unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
void PIDCallback(const lino_msgs::PID& pid);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("LINOBASE CONNECTED");

  delay(5);
}

void loop()
{
  //this block drives the robot based on defined rate
  if ((millis() - g_prev_control_time) >= (1000 / COMMAND_RATE))
  {
    moveBase();
    g_prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400)
  {
    stopBase();
  }

  //call all the callbacks waiting to be called
  nh.spinOnce();
}

void PIDCallback(const lino_msgs::PID& pid)
{
  //callback function every time PID constants are received from lino_pid for tuning
  //this callback receives pid object where P,I, and D constants are stored
  motor1_pid.updateConstants(pid.p, pid.i, pid.d);
  motor2_pid.updateConstants(pid.p, pid.i, pid.d);
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_angular_vel_z = cmd_msg.angular.z;

  g_prev_command_time = millis();
}

void moveBase()
{
  Kinematics::output req_pwm;
  //get the required rpm for each motor based on required velocities
  req_pwm = kinematics.getPWM(g_req_linear_vel_x, 0.0, g_req_angular_vel_z);
  //throw the pwm values to the motor driver
  motor1.spin(req_pwm.motor1);
  motor2.spin(req_pwm.motor2);

  /*
  //PID control can be done in this setup on a robot level by:
  //create these objects before void_setup();
  PID x_pid(-MAX_X, MAX_X, K_Px, K_Ix, K_Dx);
  PID z_pid(-MAX_Z, MAX_Z, K_Pz, K_Iz, K_Dz);

  //These are sample data, replace the values with your external odom velocities
  double ex_linear_vel = EXTERNAL_ODOM_X;
  double ex_angular_vel = EXTERNAL_ODOM_Z;

  //calculate required velocites using PID - Twist velocities vs external odometry velocities
  double pid_linear_vel = x_pid.compute(g_req_linear_vel_x, ex_linear_vel);
  double pid_angular_vel = z_pid.compute(g_req_angular_vel_z, ex_angular_vel);

  Kinematics::output req_pwm;
  //calculate pwm values for the motor based on calculated velocities using PID
  req_pwm =  kinematics.getPWM(pid_linear_vel, 0.0, pid_angular_vel);

  //throw the pwm values to the motor driver
  motor1.spin(req_pwm.motor1);
  motor2.spin(req_pwm.motor2);
  */
}

void stopBase()
{
  motor1.spin(0);
  motor2.spin(0);
}
