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

#include "battery.h"

Battery::Battery(uint8_t pin, float conversion_factor)
    : pin_(pin), conversion_factor_(conversion_factor), value_(-1.0)
{

}

// Lower bound under which measurements are no longer considered valide (~3.8V)
const uint16_t measurement_threshold = 250;

void Battery::Update()
{
  int current_value = analogRead(pin_);
  if (current_value < measurement_threshold)
  {
    value_ = 0;
    return;
  }
  if (value_ <= measurement_threshold)
  {
    value_ = current_value;
    return;
  }

  // Smoothing factor
  const float alpha = 0.1;
  value_= alpha * current_value + (1 - alpha) * value_;
}

float Battery::Voltage()
{
  if (value_ < measurement_threshold)
  {
    return 0;
  }
  return conversion_factor_ * value_ / 1024.0;
}
