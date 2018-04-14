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
 * speedsensor.cpp
 * Gigatron motor control Arduino code for Hall Effect sensors.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 * @date    2016-03-27    syler   fixed RPM calculation for new quadrature encoders and cleaned up encoder interrupt pin setup
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

#define PULSES_PER_REV 600.0 //$ number of encoder pulses per full motor revolution

SpeedSensor::SpeedSensor(int interrupt, int poles, int interval) {
  _interrupt = interrupt;
  _poles = poles;
  _interval = interval;
  //interval is set in gigatron.ino; it is the interval at which the context loop runs, 
  //and not related to any property of the encoders

  //see https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt documentation
  //in summary, attachInterrupt(4) likely attaches an interrupt to pin 18, and does not attach one to pin 4

  /*$ actually, the link above says the Mega2560 has the following mapping:
      Interrupt     0   1   2   3   4   5
      Mega2560 pin  2   3   21  20  19  18
  */

  pinMode(L_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(L_ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(R_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(R_ENCODER_PIN_B, INPUT_PULLUP);

  if (_interrupt == R_ENCODER_INTERRUPT) {
    attachInterrupt(R_ENCODER_INTERRUPT, RightEncoderISR, FALLING);
  } else {
    attachInterrupt(L_ENCODER_INTERRUPT, LeftEncoderISR, FALLING);
  }
  
  _ticks_left = _ticks_right = 0;
}

//$ returns number of ticks per S_LOOP_INTERVAL
long SpeedSensor::GetTicks() {
  long ticks;

  if (_interrupt == L_ENCODER_INTERRUPT) { // If we are the left sensor
    ticks = _ticks_left;
    _ticks_left = 0;
  } else if (_interrupt == R_ENCODER_INTERRUPT) { // right sensor
    ticks = _ticks_right;
   _ticks_right = 0;
  }

  return ticks;
}


//$ TODO: this is unsigned, need to fix!
long SpeedSensor::GetRPM() {
  long ticks;

  if (_interrupt == L_ENCODER_INTERRUPT) {// If we are the left sensor
    ticks = _ticks_left;
    _ticks_left = 0;
  } else if (_interrupt == R_ENCODER_INTERRUPT) { // right sensor
    ticks = _ticks_right;
    _ticks_right = 0;
  }
  
  double motor_revs = (double) ticks / PULSES_PER_REV;
  // double wheel_revs = motor_revs * gearRatio;
  // double rpm = wheel_revs * (60.0 * 1000) / _interval;
  double rpm = motor_revs * (60.0 * 1000) / _interval;

  return (long) rpm; 

  /*$ old code that does not apply to quadrature encoders
      plus DGonz's filter
      //Serial.println(sp);
      //Ticks *60000 millis/minute*(1/interval)*revolutions/ 7 ticks = RPM
      //long hz = sp * 1000 / _interval;
      //long rpm = 120 * hz / _poles;

      //TODO how does 600 pulses per motor revolution affect this code?

      double rpm = sp*60.0*1000.0/_interval/7.0;
      rpmSmooth = (rpm * (1 - filterVal)) + (rpmSmooth  *  filterVal);
      //Serial.println(rpmSmooth);
      if (rpmSmooth < 0) rpmSmooth = 0;
    //  dp(rpmSmooth);
      return (unsigned int) rpmSmooth;

  */

}

