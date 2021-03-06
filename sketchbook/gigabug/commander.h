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
 * commander.h
 * Gigatron motor control Arduino code Jetson and RC commanders.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner  <syler@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 *
 * @date    2015-09-16    syler   fixed odometry message sending
 **/

#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "classes.h"
#include "isr.h"
#include <ros.h>

class Commander {
public:
  virtual int GetLeftRPMCmd() {return 0;}
  virtual int GetRightRPMCmd() {return 0;}
  virtual unsigned char GetAngleCmd() {return 0;}
  virtual unsigned char GetKillCmd() {return 0;}
};

class RCCommander: public Commander {
public:
  RCCommander(RCDecoder *sp, RCDecoder *pos,  RCDecoder *kill);
  int GetLeftRPMCmd();
  int GetRightRPMCmd();
  unsigned char GetAngleCmd();
  unsigned char GetKillCmd();

private:
  RCDecoder *_sp, *_pos, *_kill;
};

class JetsonCommander: public Commander { //$ wooo
public:
  JetsonCommander(ros::NodeHandle *nh);
  int GetLeftRPMCmd();
  int GetRightRPMCmd();
  unsigned char GetAngleCmd();
  // double _angle;
  unsigned char _angle;
  long _rpm_left, _rpm_right;
  
  unsigned int _autonomous;
  unsigned int _autonomy_type;
  bool _estop;
/*
  boolean _jetsonMode; //$ true for Jetson control, false for RC
  boolean _semiautomaticMode; //$ true for Jetson control, false for RCprivate:
*/
  ros::NodeHandle *_nh;
};
  
#endif


