/* Copyright (c) 2016, Malte Splietker
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

#include "motor.h"

#include <Arduino.h>

/* Min duty cycle depending on voltage:
 * Voltage | Min Duty Cycle
 * --------+----------------
 * 5V      | 0.48
 * 7.4V    | 0.28
 * 9V      | 0.21
 * 11.1V   | 0.16
 * 12V     | 0.15
 */

const float Motor::min_duty_cycle = 0.16;

const float Motor::max_duty_cycle = 0.9;

Motor::Motor(int pin_a, int pin_b) :
    pin_a_(pin_a), pin_b_(pin_b), speed_(0), direction_(Motor::FORWARD)
{
  Write();
}

Motor::~Motor()
{}

void Motor::SetSpeed(float speed)
{
  float abs_speed = fabs(speed);
  if (abs_speed > 1)
  {
    abs_speed = 1;
  }
  else if (speed >= 0)
  {
    direction_ = Motor::FORWARD;
  }
  else
  {
    direction_ = Motor::BACKWARD;
  }
  speed_ = abs_speed;

  Write();
}

void Motor::Write()
{
  float actual_duty_cycle = (min_duty_cycle + speed_ * (max_duty_cycle - min_duty_cycle));
  uint8_t pwm_value = (uint8_t) (255 * actual_duty_cycle);

  if (speed_ == 0)
  {
    pwm_value = 0;
  }

  if (direction_ == Motor::FORWARD)
  {
    analogWrite(pin_a_, pwm_value);
    analogWrite(pin_b_, 0);
  }
  else
  {
    analogWrite(pin_a_, 0);
    analogWrite(pin_b_, pwm_value);
  }
}

PIDController::PIDController(Motor *motor)
    : motor_(motor),
      controller_(0.5, 0.55, 0.001),
      input_(0), output_(0), set_speed_(0), direction_(1)
{

}

void PIDController::Init()
{
}

void PIDController::EncoderUpdate(double value)
{
  controller_.Update(value);
  if (fabs(set_speed_) >= 0.001)
  {
    motor_->SetSpeed(controller_.output());
  }
  else
  {
    motor_->SetSpeed(0);
  }
}

void PIDController::SetSpeed(double speed)
{
  set_speed_ = speed;
  controller_.setpoint(speed * 4);
}

const double PIDController::set_speed() const
{
  return set_speed_;
}
