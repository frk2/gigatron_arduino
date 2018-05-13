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
 * dcservo.cpp
 * Gigatron motor control Arduino code for steering servo.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "classes.h"
#include "isr.h"

DCServo::DCServo(int pwmPin1, int pwmPin2, int posPin) {
  _pwmPin1 = pwmPin1;
  _pwmPin2 = pwmPin2;
  _posPin = posPin;
  _minV = 378;
  _midV = 574;
  _maxV = 800;
  
  pinMode(_pwmPin1, OUTPUT);
  pinMode(_pwmPin2, OUTPUT);
  pinMode(_posPin, INPUT);
}

void DCServo::ConfigPot(int minV, int midV, int maxV) {
  _minV = minV;
  _midV = midV;
  _maxV = maxV;
}

void DCServo::SetVelocity(int vel) {
  //if (vel>200) vel = 200; //200 is my cap
  
  if (vel < 0) {
    analogWrite(_pwmPin1, -vel);
    digitalWrite(_pwmPin2, LOW);
  } else {
    analogWrite(_pwmPin2, vel);
    digitalWrite(_pwmPin1, LOW);
  }
}

/*$ DEPRECATED */
unsigned char DCServo::GetPos() {
  long adu = analogRead(_posPin);
  long tmp = (adu - _minV) << 8 ;;
  tmp /= (_maxV - _minV);
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
//  dp(tmp);
  return (unsigned char) tmp;
}

//$ takes pot limits and middle value and linearizes the output
unsigned char DCServo::GetPosLinearized() {
  long adu = analogRead(_posPin);
  //dp(adu); //$ uncomment for pot calibration
  long tmp;
  if (adu < _midV) {
    tmp = (adu - _minV) << 7 ;;
    tmp /= (_midV - _minV);
  }
  else {
    tmp = (adu - _midV) << 7 ;
    tmp /= (_maxV - _midV);
    tmp += 127;
  }
  if (tmp < 0) tmp = 0;
  if (tmp > 255) tmp = 255;
  //dp(tmp);
  unsigned char res = (unsigned char) tmp;
//  dp (res);
  return res;
}

