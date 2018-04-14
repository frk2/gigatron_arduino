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
 * logicbatterysensor.cpp
 * Gigatron sensor code for logic battery level.
 *
 * @author Chris Desnoyers <cjdesno@mit.edu>
 *
 * @date 2017-01-21
 *
 **/

 #include <Arduino.h>
 #include "classes.h"

 #define ADC_TO_PIN_VOLTAGE (5./1024.)

LogicBatterySensor::LogicBatterySensor(int sense_pin, long r_top, long r_bottom) {
  _sense_pin = sense_pin;
  _r_top = r_top;
  _r_bottom = r_bottom;
  pinMode(_sense_pin, INPUT);
}

double LogicBatterySensor::GetLogicVoltage() {
  int pin_reading = analogRead(_sense_pin);
  return pin_reading * ADC_TO_PIN_VOLTAGE * (_r_top + _r_bottom)/((double) _r_bottom);
}