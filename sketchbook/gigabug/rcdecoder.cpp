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
 * rcdecoder.cpp
 * Gigatron motor control Arduino code for RC decoder.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

RCDecoder::RCDecoder(int interrupt, int minV, int maxV) {
  _interrupt = interrupt;
  _minV = minV;
  _maxV = maxV;
  
  
  if (_interrupt == RC_STEERING_INTERRUPT) {
    attachInterrupt(RC_STEERING_INTERRUPT, RCSteeringISR, CHANGE);
  } else if (_interrupt == RC_THROTTLE_INTERRUPT) {
    attachInterrupt(RC_THROTTLE_INTERRUPT, RCThrottleISR, CHANGE);
  } else if (_interrupt == RC_KILL_INTERRUPT) {
    attachInterrupt(RC_KILL_INTERRUPT, RCKillISR, CHANGE);
  }
}

unsigned char RCDecoder::GetVal() {
  long pw;
  if (_interrupt == RC_STEERING_INTERRUPT) {
    pw = _pw0_us;
  } else if (_interrupt == RC_THROTTLE_INTERRUPT) {
    pw = _pw1_us;
  }if (_interrupt == RC_KILL_INTERRUPT) {
    pw = _pw2_us;
    if (pw <= _minV) pw = 0;
    if (pw > _maxV) pw = 255;
    return pw; 
  }
  
  //Serial.println(pw); //RC decoder vals
  pw = (pw - _minV) << 8;
  pw /= (_maxV - _minV);
  if (pw < 0) pw = 0;
  if (pw > 255) pw = 255;
  return (unsigned char) pw;
}

