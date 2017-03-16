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

#ifndef AS5040_AS5040_H
#define AS5040_AS5040_H


#include "Arduino.h"

class AS5040
{
public:
  AS5040(uint8_t pin_DO, uint8_t pin_CLK, uint8_t pin_CS);

  void Init();

  uint16_t Read();

  uint16_t Value();

  bool IsValid();

  bool GetStatus(uint8_t field);

  enum Status
  {
    /**
     * Magnitude decreased (magnet pushed towards IC)
     */
    STATUS_MAG_DEC = 0,
    /**
     * Magnitude increased (magnet pulled away from IC)
     */
    STATUS_MAG_INC,
    /**
     * Linearity alarm (output linearity critical)
     */
    STATUS_LIN,
    /**
     * CORDIC Overflow (data is invalid)
     */
    STATUS_COF,
    /**
     * Offset Compensation Finished (AS5040 is ready)
     */
    STATUS_OCF
  };

private:
  inline bool ParityValid(uint16_t data);

  /**
   * Last 16 bits read from the AS5040.
   */
  uint16_t last_data_;

  /**
   * Data output of the AS5040.
   */
  uint8_t pin_DO_;

  /**
   * Clock pin.
   */
  uint8_t pin_CLK_;

  /**
   * Chip select pin.
   */
  uint8_t pin_CS_;


};


#endif //AS5040_AS5040_H
