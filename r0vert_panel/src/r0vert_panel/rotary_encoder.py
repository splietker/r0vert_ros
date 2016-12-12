# Copyright (c) 2016, Malte Splietker
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * The names of contributors may not be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import RPi.GPIO as GPIO
import time


class Flank:
    NONE = 0
    RISING = 1
    FALLING = 2


class Direction:
    CCW = -1
    CW = 1


class EventType:
    ROTATION = 0
    BUTTON = 1


class RotaryEncoder:
    def __init__(self, pin_a, pin_b, pin_sw):
        self.__pin_a = pin_a
        self.__pin_b = pin_b
        self.__pin_sw = pin_sw

        self.__switch_state = 0

        self.__direction = 0
        self.__state = 0
        self.__fail_count = 0
        self.__state_pin_a = 0
        self.__state_pin_b = 0

        self.__callbacks = {
            EventType.ROTATION: [],
            EventType.BUTTON: []
        }

        GPIO.setmode(GPIO.BCM)  # Numbers GPIOs by physical location
        GPIO.setup(self.__pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.__pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.__pin_sw, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.__pin_a, GPIO.BOTH, callback=self.__rotation_callback, bouncetime=2)
        GPIO.add_event_detect(self.__pin_b, GPIO.BOTH, callback=self.__rotation_callback, bouncetime=2)
        GPIO.add_event_detect(self.__pin_sw, GPIO.BOTH, callback=self.__push_button_callback, bouncetime=20)

    def __del__(self):
        GPIO.cleanup([self.__pin_a, self.__pin_b, self.__pin_sw])

    def register_callback(self, event_type, callback):
        self.__callbacks[event_type].append(callback)

    def __perform_callback(self, event_type, value):
        for function in self.__callbacks[event_type]:
            function(value)

    def __update_state(self, channel, flank):
        if self.__fail_count >= 20:
            self.__state = 0
            self.__fail_count = 0

        if self.__state == 0 and flank == Flank.RISING:
            if channel == self.__pin_a:
                self.__direction = Direction.CW
            else:
                self.__direction = Direction.CCW
        elif self.__state == 1 and flank == Flank.RISING:
            if channel == self.__pin_a and self.__direction == Direction.CCW:
                pass
            elif channel == self.__pin_b and self.__direction == Direction.CW:
                pass
            else:   # Error
                self.__fail_count += 1
                return
        elif self.__state == 2 and flank == Flank.FALLING:
            if channel == self.__pin_a and self.__direction == Direction.CW:
                pass
            elif channel == self.__pin_b and self.__direction == Direction.CCW:
                pass
            else:   # Error
                self.__fail_count += 1
                return
        elif self.__state == 3 and flank == Flank.FALLING:
            if channel == self.__pin_a and self.__direction == Direction.CCW:
                pass
            elif channel == self.__pin_b and self.__direction == Direction.CW:
                pass
            else:   # Error
                self.__fail_count += 1
                return
        else:
            self.__fail_count += 1
            return
        self.__state += 1

        if self.__state == 4:
            self.__state = 0
            self.__perform_callback(EventType.ROTATION, self.__direction)

    def __rotation_callback(self, channel):
        # Additional debounce time
        time.sleep(0.002)
        value = GPIO.input(channel)

        # Flank detection
        flank = Flank.NONE
        if channel == self.__pin_a:
            if self.__state_pin_a - value == 1:     # Pin A RISE
                flank = Flank.RISING
            elif self.__state_pin_a - value == -1:  # Pin A FALL
                flank = Flank.FALLING
            self.__state_pin_a = value
        elif channel == self.__pin_b:
            if self.__state_pin_b - value == 1:     # Pin B RISE
                flank = Flank.RISING
            elif self.__state_pin_b - value == -1:  # Pin B FALL
                flank = Flank.FALLING
            self.__state_pin_b = value

        if flank == Flank.NONE:
            return

        self.__update_state(channel, flank)

    def __push_button_callback(self, channel):
        value = GPIO.input(channel)
        if value == 0 and self.__switch_state == 0:
            self.__switch_state = 1
            self.__perform_callback(EventType.BUTTON, 1)
        elif value == 1 and self.__switch_state == 1:
            self.__switch_state = 0
            self.__perform_callback(EventType.BUTTON, 0)
