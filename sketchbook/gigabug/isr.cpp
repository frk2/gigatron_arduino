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
 * isr.cpp
 * Gigatron motor control Arduino code for interrupt service routines.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 *
 * @date    2016-03-27    syler   define encoder interrupts and pins, fixed ISRs for left and right encoders
 *
 **/

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "isr.h"

volatile unsigned long _pw0_us, _pw1_us, _pw2_us;
volatile unsigned long _pw0_last_t, _pw1_last_t, _pw2_last_t;

volatile long _ticks_left, _ticks_right;  //$ number of ticks for each encoder
volatile bool _read_left, _read_right;    //$ state of digitalRead(encoder pin B) for each encoder

void RCSteeringISR() {
  int state = digitalRead(2);
  if (state) {
    _pw0_last_t = micros();
  } else {
    _pw0_us = micros() - _pw0_last_t;
  }
}

void RCThrottleISR() {
  int state = digitalRead(3);
  if (state) {
    _pw1_last_t = micros();
  } else {
    _pw1_us = micros() - _pw1_last_t;
  }
}

void RCKillISR() {
  int state = digitalRead(21);
  if (state) {
    _pw2_last_t = micros();
  } else {
    _pw2_us = micros() - _pw2_last_t;
  }
}

//Adapted for quadrature encoder, direction inferred from pulse alignment (11 forward, 10 backward)
//$ left encoder interrupt service routine
void LeftEncoderISR() { 
  _read_left = digitalReadFast(L_ENCODER_PIN_B); //$ read encoder input pin B

  #ifdef L_ENCODER_REVERSED //$ if left encoder is reversed
    //$ increment counter if B leads A
    _ticks_left += _read_left ? -1 : +1; //$ if (_read_left) {_ticks_left--;} else {_ticks_left++;}
  #else
    //$ increment counter if A leads B
    _ticks_left += _read_left ? +1 : -1; //$ if (_read_left) {_ticks_left++;} else {_ticks_left--;}
  #endif

}

//$ right encoder interrupt service routine
void RightEncoderISR() { 
  _read_right = digitalReadFast(R_ENCODER_PIN_B); //$ read encoder input pin B

  #ifdef R_ENCODER_REVERSED //$ if right encoder is reversed
    //$ increment counter if B leads A
    _ticks_right += _read_right ? -1 : +1; //$ if (_read_right) {_ticks_right++;} else {_ticks_right--;}
  #else
    //$ increment counter if A leads B
    _ticks_right += _read_right ? +1 : -1; //$ if (_read_right) {_ticks_right--;} else {_ticks_right++;}
  #endif

}





