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
 * context.h
 * Gigatron motor control Arduino code.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 * @date    2016-01-10    syler   moved PID controller to separate class
 *
 **/

#ifndef __CONTEXT_H
#define __CONTEXT_H

#include <Arduino.h>
#include "isr.h"
#include "classes.h"
#include "commander.h"
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

//$ motor commands
#include <gigatron_msgs/MotorCommand.h>

//$ debugging messages
#include <gigatron_msgs/Radio.h>
#include <gigatron_msgs/Steering.h>
#include <gigatron_msgs/Motors.h>
#include <std_msgs/UInt8.h>

class Context {
public:
  Context(Commander *commander, DCServo *servo,
          SpeedSensor *left, SpeedSensor *right,
          int lPwm, int rPwm,
          int lRev, int rRev,
          PIDController *lSp, PIDController *rSp,
          PIDController *pos,
          ros::NodeHandle *nh,
          JetsonCommander *jcommander,
          LogicBatterySensor *logic_bat,
          gigatron_msgs::Radio *radio_msg,
          ros::Publisher *radio_pub,
          gigatron_msgs::Steering *steer_msg,
          ros::Publisher *steer_pub,
          gigatron_msgs::Motors *mot_msg,
          ros::Publisher *mot_pub,
          std_msgs::UInt8 *stop_msg,
          ros::Publisher *stop_pub,
          std_msgs::Float32 *voltage_msg,
          ros::Publisher *voltage_pub
          );
  void ConfigureLoop(int sInterval, int pInterval, int pubInterval);
  void Start();

private:
  Commander *_commander;
  DCServo *_servo;
  SpeedSensor *_left, *_right;
  int _lPwm, _rPwm, _lRev, _rRev;
  PIDController *_lSp, *_rSp, *_pos;
  int _sInterval, _pInterval, _pubInterval;
  LogicBatterySensor *_battery_sensor;
  
  unsigned long _last_st, _last_pt, _last_pub;

  //$
  ros::NodeHandle *_nh;
  JetsonCommander *_jcommander;

  gigatron_msgs::Radio *_radio_msg;
  ros::Publisher *_radio_pub;

  gigatron_msgs::Steering *_steer_msg;
  ros::Publisher *_steer_pub;

  gigatron_msgs::Motors *_mot_msg;
  ros::Publisher *_mot_pub;

  std_msgs::UInt8 *_stop_msg;
  ros::Publisher *_stop_pub;

  std_msgs::Float32 *_voltage_msg;
  ros::Publisher *_voltage_pub;

};

#endif


