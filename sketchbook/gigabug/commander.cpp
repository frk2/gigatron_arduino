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
 * commander.cpp
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include "commander.h"


RCCommander::RCCommander(RCDecoder *sp, RCDecoder *pos, RCDecoder *kill) {
  _sp = sp;
  _pos = pos;
  _kill = kill;
}

int RCCommander::GetLeftRPMCmd() { 
  if (_sp->GetVal() < 50) return 0;
  int left_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (left_command > 250) left_command = 250;
  if (left_command < -250) left_command = -250;
  
  return left_command;
}

int RCCommander::GetRightRPMCmd() {
  if (_sp->GetVal() < 50) return 0;  
  int right_command = 2 * (int) _sp->GetVal() - 255;

  //$ check edge cases
  if (right_command > 250) right_command = 250;
  if (right_command < -250) right_command = -250;
  
  return right_command;
}

unsigned char RCCommander::GetAngleCmd() {
  return _pos->GetVal();
}

unsigned char RCCommander::GetKillCmd() {
  return _kill->GetVal();
}

//$
JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {

  _nh = nh;

  _autonomous = 0;
  _autonomy_type = 2;

  _estop = false;

  _rpm_left = 0; 
  _rpm_right = 0; 
  _angle = 128;
}

int JetsonCommander::GetLeftRPMCmd() {
  return (int) _rpm_left;
}

int JetsonCommander::GetRightRPMCmd() {
    return (int) _rpm_right;
}

unsigned char JetsonCommander::GetAngleCmd() {
  return _angle;
}



