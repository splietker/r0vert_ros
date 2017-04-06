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

#ifndef R0VERT_FIRMWARE_WHEEL_ENCODER_H
#define R0VERT_FIRMWARE_WHEEL_ENCODER_H


#include <Arduino.h>
#include "as5040.h"

class WheelEncoder
{
public:
  WheelEncoder(uint8_t pin_DO, uint8_t pin_CLK, uint8_t pin_CS);

  void Init();

  void Update();

  /**
   * Calculates the current rotation velocity (in turns/s).
   *
   * Calculates the rotation velocity as observed since the last call. The time between consecutive calls should
   * not be too long as the readings could be erroneous otherwise.
   * @return Current rotation velocity.
   */
  double Velocity();

private:
  AS5040 sensor_;

  double last_sensor_value_;

  unsigned long last_velocity_calculation_time_;

  double velocity_turn_sum_;
};


#endif //R0VERT_FIRMWARE_WHEEL_ENCODER_H
