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

#include <ros.h>
#include "wheel_encoder.h"

WheelEncoder::WheelEncoder(uint8_t pin_DO, uint8_t pin_CLK, uint8_t pin_CS, bool inverted)
    : sensor_(pin_DO, pin_CLK, pin_CS), inverted_(inverted)
{

}

void WheelEncoder::Init()
{
  sensor_.Init();
}

/**
 * Calculates the minimum distance between two turn values in [-1,1].
 *
 * The distance between both turn value in [0, 1] is calculated by keeping the minimum difference of both possible
 * directions. The direction of the minimum distance is encoded as follows:
 *  > 0 means CW
 *  < 0 means CCW
 * @param value1
 * @param value2
 * @return
 */
double turn_distance(double value1, double value2)
{
  double diff = value1 - value2;
  // Return the minimum of both directions
  if (diff > 0.5)
  {
    return 1 - diff;
  }
  else if (diff < -0.5)
  {
    return -1 - diff;
  }
  return -diff;
}

void WheelEncoder::Update()
{
  double sensor_value = sensor_.Read() / 1024.0;
  velocity_turn_sum_ += turn_distance(sensor_value, last_sensor_value_);
  last_sensor_value_ = sensor_value;
}

double WheelEncoder::Velocity()
{
  extern ros::NodeHandle nh;
  unsigned long current_time = micros();

  double current_velocity = velocity_turn_sum_ / ((current_time - last_velocity_calculation_time_) * 1e-6);

  last_velocity_calculation_time_ = current_time;
  velocity_turn_sum_ = 0;
  return inverted_ ? -current_velocity : current_velocity;
}
