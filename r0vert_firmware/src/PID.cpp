/* Copyright (c) 2017, Malte Splietker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The names of contributors may not be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include <ros.h>

#include "PID.h"

PID::PID(double Kp, double Ki, double Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd)
{

}

double PID::Update(double measurement)
{
  unsigned long now = micros();
  double time_diff = (now - previous_update_time_) * 1e-6;

  double error = setpoint_ - measurement;
  integral_ += error * time_diff;
  // Bounds against integral windup
  integral_ = min(integral_, 10);
  integral_ = max(integral_, -10);
  double derivative = (error - previous_error_) / time_diff;

  output_ = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);
  output_ = min(output_, 1);
  output_ = max(output_, -1);
  if (setpoint_ < 0)
  {
    output_ = min(output_, 0);
  }
  else
  {
    output_ = max(output_, 0);
  }

  previous_error_ = error;
  previous_update_time_ = now;
  return output_;
}

void PID::setpoint(double setpoint)
{
  setpoint_ = setpoint;
}

double PID::output()
{
  return output_;
}
