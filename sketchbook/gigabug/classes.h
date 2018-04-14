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
 * pidcontroller.cpp
 * Gigatron motor control Arduino code for class definitions.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   refactored header files
 *
 **/

#ifndef __CLASSES_H
#define __CLASSES_H

#define dp(var) Serial.print(#var);Serial.print(": ");Serial.println(var)


#define PI 3.1415926535897932384626433832795
#define INCHES_TO_M 0.0254 //$ conversion from inches to meters
//$ car dimensions
const static double wheelBaseWidth = 23.0 * INCHES_TO_M;  //$ [m]
const static double wheelRadius = 4.90 * INCHES_TO_M;     //$ [m]
const static double gearRatio = 11.0 / 60.0;  //$ gear ratio between motor and wheels

//$ constants
const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second

const static double STEERING_PWM_RANGE = 255.0;
const static double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
const static double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]


class DCServo {
public:
  DCServo(int pwmPin1, int pwmPin2, int posPin);
  void ConfigPot(int minV, int midV, int maxV);
  void SetVelocity(int vel);
  unsigned char GetPos();
  unsigned char GetPosLinearized();
private:
  int _pwmPin1, _pwmPin2, _posPin;
  int _minV, _midV, _maxV;
};

class RCDecoder {
public:
  RCDecoder(int interrupt, int minV, int maxV); // edges of pulse widths in microseconds
  unsigned char GetVal();
private:
  int _interrupt;
  int _minV, _maxV;
};

class SpeedSensor {
public:
  SpeedSensor(int interrupt, int poles, int interval);
  long GetTicks();
  long GetRPM();
  double rpmSmooth = 0.0;
  double filterVal = 0.25;
private:
  int _interrupt;
  int _poles, _interval;
};

class PIDController {
public:
  PIDController(long kp, long ki, long kd, long out_max, long out_min);
  int Update(int ref, int in);
  void ResetIntegrator();
  void ResetGains(long kp, long ki, long kd);
private:
  long _kp, _ki, _kd;
  long _out_max, _out_min;
  
  int _last_in;
  long _integral;
};

class LogicBatterySensor {
public:
  LogicBatterySensor(int sense_pin, long r_top, long r_bottom);
  double GetLogicVoltage();
private:
  int _sense_pin;
  long _r_top;
  long _r_bottom;
};
  

#endif


