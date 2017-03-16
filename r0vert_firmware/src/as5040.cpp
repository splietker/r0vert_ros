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

#ifndef R0VERT_FIRMWARE_AS5040_H
#define R0VERT_FIRMWARE_AS5040_H

#include "as5040.h"

AS5040::AS5040(uint8_t pin_DO, uint8_t pin_CLK, uint8_t pin_CS) : pin_DO_(pin_DO), pin_CLK_(pin_CLK), pin_CS_(pin_CS)
{}

void AS5040::Init()
{
  pinMode(pin_DO_, INPUT_PULLUP);
  pinMode(pin_CLK_, OUTPUT);
  pinMode(pin_CS_, OUTPUT);

  // Set high to reset AS5040 output_left
  digitalWrite(pin_CS_, HIGH);
  digitalWrite(pin_CLK_, HIGH);
}

uint16_t AS5040::Read()
{
  digitalWrite(pin_CS_, LOW);

  // Update upper and lower bytes
  uint16_t data = shiftIn(pin_DO_, pin_CLK_, MSBFIRST) << 8;
  data |= shiftIn(pin_DO_, pin_CLK_, MSBFIRST);

  digitalWrite(pin_CS_, HIGH);

  last_data_ = data;

  return Value();
}

uint16_t AS5040::Value()
{
  return last_data_ >> 6;
}

bool AS5040::IsValid()
{
  if (not ParityValid(last_data_))
  {
    Serial.println();
    Serial.print("R1");
    return false;
  }
  else if (GetStatus(STATUS_MAG_DEC) && GetStatus(STATUS_MAG_INC))
  {
    Serial.println();
    Serial.print("R2");
    return false;
  }
  else if (GetStatus(STATUS_COF))
  {
    Serial.println();
    Serial.print("R3");
    return false;
  }

  return true;
}

bool AS5040::GetStatus(uint8_t field)
{
  return (last_data_ & (1 << (1 + field))) != 0;
}

bool AS5040::ParityValid(uint16_t data)
{
  uint16_t mask = 0x8000;
  uint8_t one_bits = 0;
  while (mask >= 0x0001)
  {
    if (data & mask)
    {
      one_bits += 1;
    }
    mask = mask >> 1;
  }

  return (one_bits % 2 == 0);
}

#endif //R0VERT_FIRMWARE_AS5040_H
