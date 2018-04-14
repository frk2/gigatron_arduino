/********************************************************************
  Software License Agreement (BSD License)

  Copyright (c) 2017, Cult Classic Racing.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

/**
   gigabug.ino
   Gigatron motor control debugging Arduino code.

   @author  Bayley Wang       <bayleyw@mit.edu>
   @author  Syler Wagner      <syler@mit.edu>
   @author  Chris Desnoyers   <cjdesno@mit.edu>
   @author  Daniel Gonzalez   <dgonz@mit.edu>

   @date    2016-03-12    syler   creation with custom Arduino debugging message formats

 **/

#include <Servo.h>
#include <digitalWriteFast.h>
#include "classes.h"
#include "commander.h"
#include "context.h"
#include "isr.h"
#define USE_USBCON
#include <ros.h>

#include <geometry_msgs/Vector3.h>  //$ for gain adjustment
#include <std_msgs/UInt8.h>         //$ for mode publishing
#include <std_msgs/Bool.h>          //$ for estop subscriber

//$ motor commands
#include <gigatron_msgs/MotorCommand.h>

//$ debugging messages
#include <gigatron_msgs/Radio.h>
#include <gigatron_msgs/Steering.h>
#include <gigatron_msgs/Motors.h>
#include <gigatron_msgs/Motors.h>
#include <std_msgs/Float32.h>

#define LOOP_INTERVAL 10
#define S_LOOP_INTERVAL 100
#define PUB_INTERVAL 100

//$ steering pot calibration 2017-01-21
int minADU = 493; //$ max right
int midADU = 598; //$ value at zero steering angle
int maxADU = 675; //$ max left

ros::NodeHandle nh;       //$ node handle

// JetsonCommander(ros::NodeHandle *nh);
JetsonCommander jc(&nh);  //$ Jetson commander

//PIDController(long kp, long ki, long kd, long out_max, long out_min)
//Note from DGonz: Low kP, some kI and no kD is best for wheel velocities. 
PIDController lSp(40, 8, 1, 500, -500); //$ left drive motor PID controller
PIDController rSp(40, 8, 1, 500, -500); //$ right drive motor PID controller

/*$ The PID controllers for the drive motors are only active in AUTO mode.
 */

PIDController pPos(150, 0, 15, 255, -255); //$ steering servo PID controller

gigatron_msgs::Radio radio_msg;
gigatron_msgs::Steering steer_msg;
gigatron_msgs::Motors mot_msg;
std_msgs::UInt8 mode_msg;
std_msgs::Float32 voltage_msg;

LogicBatterySensor logic_bat(A3, 270000, 56000);

void CmdCallback(const gigatron_msgs::MotorCommand& cmd) {
  jc._angle = cmd.angle_command;
  jc._rpm_left = cmd.rpm_left;
  jc._rpm_right = cmd.rpm_right;
}

/*$ 
  Enable lidar-based estop. 
*/
void StopCallback(const std_msgs::Bool& mode) {
    jc._estop = mode.data;
}

/*$ 
  Change default autonomy mode
*/
void ModeCallback(const std_msgs::UInt8& mode) {
  if (mode.data == 1)
  {
    jc._autonomy_type = 1; //$ SEMIAUTOMATIC
  }
  else if (mode.data == 2)
  {
    jc._autonomy_type = 2; //$ 2AUTO4U
  }
  /*$ 
  Note that any other value will have no effect, if the 
  mode has value 0 that will get overridden by the RC
  kill witch. This callback only toggles the default 
  autonomy type, not necessarily whether or not the car 
  is autonomous.
  */

  if (jc._autonomous > 0)
  {
    /*$ 
    If the car is not in RC mode when the callback occurs,
    we want the change to take effect immediately.
    */
    jc._autonomous = jc._autonomy_type;
  }
}


/*$ 
  Set PID controller gains for both drive motors with a
  Vector3 ROS message (kp, ki, kd) published on the /gains
  topic.
*/
/* DEPRECATED */
void GainsCallback(const geometry_msgs::Vector3& gain) {
  long kp = (long) gain.x;
  long ki = (long) gain.y;
  long kd = (long) gain.z;
  lSp.ResetGains(kp, ki, kd);
  rSp.ResetGains(kp, ki, kd);
}

void setup() {

/*$
   For some reason rosserial_arduino breaks if you do both 
   Serial.begin(<BAUD>) and nh.initNode(). So either do:
   1. Serial.begin(<BAUD>)
   2. nh.getHardware()->setBaud(38400);
      nh.initNode();
   Both of the two options seem to work equally well.
  */
  Serial.begin(115200);

  //$ set up publishers
  ros::Publisher radio_pub("arduino/radio", &radio_msg);
  nh.advertise(radio_pub);
  ros::Publisher mot_pub("arduino/motors", &mot_msg);
  nh.advertise(mot_pub);
  ros::Publisher steer_pub("arduino/steering", &steer_msg);
  nh.advertise(steer_pub);
  ros::Publisher mode_pub("arduino/mode", &mode_msg);
  nh.advertise(mode_pub);
  ros::Publisher voltage_pub("arduino/voltage", &voltage_msg);
  nh.advertise(voltage_pub);

  //$ set up subscribers
  ros::Subscriber<gigatron_msgs::MotorCommand> sub("arduino/command/motors", CmdCallback);
  nh.subscribe(sub);
  ros::Subscriber<std_msgs::Bool> stop_sub("arduino/command/stop", StopCallback);
  nh.subscribe(stop_sub);
  ros::Subscriber<std_msgs::UInt8> mode_sub("arduino/command/mode", ModeCallback);
  nh.subscribe(mode_sub);

  pinMode(RC_STEERING_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  pinMode(RC_KILL_PIN, INPUT);

  // RCDecoder(int interrupt, int minV, int maxV);
  RCDecoder pos(RC_STEERING_INTERRUPT, 984, 1996);
  //Was 1480, expanded to add reverse
  RCDecoder sp(RC_THROTTLE_INTERRUPT, 1020, 1990);
  //Was 1480, expanded to add reverse
  RCDecoder kill(RC_KILL_INTERRUPT, 996, 1988);

  // SpeedSensor(int interrupt, int poles, int interval);
  SpeedSensor left(L_ENCODER_INTERRUPT, 14, S_LOOP_INTERVAL);
  SpeedSensor right(R_ENCODER_INTERRUPT, 14, S_LOOP_INTERVAL);

  // DCServo(int pwmPin1, int pwmPin2, int posPin);
  DCServo servo(STEERING_PWM_PIN_1, STEERING_PWM_PIN_2, STEERING_POT_PIN);

  // DCServo::ConfigSensor(int minV, int maxV);
  servo.ConfigPot(minADU, midADU, maxADU);
  RCCommander rc(&sp, &pos, &kill);

  /* Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int L_MOTOR_PWM_PIN, L_MOTOR_PWM_PINrPwm,
          PIDController *lSp, PIDController *rSp,
          PIDController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          gigatron_msgs::Radio *radio_msg,
          ros::Publisher *radio_pub,
          gigatron_msgs::Steering *steer_msg,
          ros::Publisher *steer_pub,
          gigatron_msgs::Motors *mot_msg,
          ros::Publisher *mot_pub
          ) */
   
  Context context(&rc, &servo, &left, &right, L_MOTOR_PWM_PIN, R_MOTOR_PWM_PIN, L_MOTOR_REVERSE_PIN, R_MOTOR_REVERSE_PIN, &lSp, &rSp, &pPos, &nh, &jc, &logic_bat, &radio_msg, &radio_pub, &steer_msg, &steer_pub, &mot_msg, &mot_pub, &stop_msg, &stop_pub, &voltage_msg, &voltage_pub);

  // Context::ConfigureLoop(int sInterval, int pInterval);
  context.ConfigureLoop(S_LOOP_INTERVAL, LOOP_INTERVAL, PUB_INTERVAL);
  //TCCR3B &= ~7;
  //TCCR3B |= 2;

  context.Start(); // the actual looping happens here
 
}

void loop() {  
}

