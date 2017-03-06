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

#ifndef _MOTOR_MOTOR_H_
#define _MOTOR_MOTOR_H_

/**
 * An H-Bride driven motor.
 *
 * Implementation for driving a motor driven by an H-Bridge (e.g. LM298). The H-Bridge needs to be connected to two PWM
 * pins (pin_a and pin_b).
 */
class Motor
{
public:
  Motor(int pin_a, int pin_b);

  ~Motor();

  /**
   * Sets the speed of the motor.
   * Sets the new speed and direction values and applies the changes to the pins. If the given value is positiv the
   * new direction is FORWARD, BACKWARD otherwise.
   *
   * @param speed Value between -1 and 1.
   */
  void SetSpeed(float speed);

private:
  /**
   * Direction of rotation.
   */
  enum Direction
  {
    FORWARD,
    BACKWARD
  };

  static const float min_duty_cycle;
  static const float max_duty_cycle;

  /**
   * Applies direction and speed to the pins.
   */
  void Write();

  int pin_a_;
  int pin_b_;
  float speed_;
  Direction direction_;
};

#endif /* _MOTOR_MOTOR_H_ */