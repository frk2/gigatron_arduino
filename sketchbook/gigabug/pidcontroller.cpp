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
 * Gigatron motor control Arduino code for PID controller.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 *
 **/

#include <Arduino.h>
#include "isr.h"
#include "classes.h"

PIDController::PIDController(long kp, long ki, long kd, long out_max, long out_min) {
  _kp = kp; 
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
  _out_max = out_max;
  _out_min = out_min;
}

void PIDController::ResetGains(long kp, long ki, long kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _last_in = 0;
  _integral = 0;
}

void PIDController::ResetIntegrator() {
  _last_in = 0;
  _integral = 0;
}

int PIDController::Update(int ref, int in) {
  //error = desired - actual
  int error = ref - in;

  //Add the error to the integral term
  _integral += error;

  //Integrator windup limiter: limit integral term to _out_max and _out_min. 
  //If the output command with only the integrator is past the limit, keep it under the limit. 
  long cmd_output = _integral * _ki >> 8;
  if (cmd_output > _out_max) _integral = (_out_max << 8) / _ki; 
  if (cmd_output < _out_min) _integral = (_out_min << 8) / _ki;

  //Get the derivative and save the current value for the next iteration
  int deriv = _last_in - in;
  _last_in = in;
  
  //add the PID terms to the command output
  cmd_output = _ki * _integral + _kp * error + _kd * deriv;

  //This bit shift is necessary for some reason
  cmd_output = cmd_output >> 8; 
  
  //Limit the command output
  if (cmd_output > _out_max) cmd_output = _out_max;
  if (cmd_output < _out_min) cmd_output = _out_min;

  //Return the command output
  return cmd_output;
}

