#!/usr/bin/env python

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


from threading import Thread, Event
import rospy
import RPi.GPIO as GPIO

from r0vert_msgs.msg import BatteryVoltage
from r0vert_panel import RotaryEncoder, EventType
from r0vert_panel.menu import MenuViewer
from r0vert_panel.status import Status

import Adafruit_CharLCD as LCD

lcd_config = {
    "rs": 23,
    "en": 24,
    "d4": 25,
    "d5": 8,
    "d6": 7,
    "d7": 12,
    "cols": 16,
    "lines": 2,
    "backlight": 18,
    "enable_pwm": True
}

rotenc_config = {
    "pin_a": 13,
    "pin_b": 19,
    "pin_sw": 26
}

status = Status()
viewer = None
rotenc = None
lcd = None
backlight_timer = None


class BackLightTimer(Thread):
    def __init__(self, timeout, lcd):
        Thread.__init__(self)
        self.timeout = timeout
        self.lcd = lcd
        self.timeout_event = Event()
        self.stop_event = Event()

    def stop(self):
        self.stop_event.set()
        self.timeout_event.set()

    def reset(self):
        lcd.set_backlight(0)
        self.timeout_event.set()

    def run(self):
        while True:
            self.timeout_event.wait(self.timeout)
            if self.stop_event.is_set():
                break
            if not self.timeout_event.is_set():
                lcd.set_backlight(1)
            else:
                self.timeout_event.clear()


def rotenc_callback(value):
    lcd.set_backlight(0);
    backlight_timer.reset()


def battery_callback(data):
    status.battery1.value = data.battery1
    status.battery2.value = data.battery2
    viewer.show()


def shutdown():
    lcd.clear()
    lcd.set_backlight(1)
    backlight_timer.stop()
    viewer.stop()
    GPIO.cleanup()


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('panel', anonymous=True)
    rospy.on_shutdown(shutdown)

    global rotenc
    rotenc = RotaryEncoder(**rotenc_config)
    rotenc.register_callback(EventType.BUTTON, rotenc_callback)
    rotenc.register_callback(EventType.ROTATION, rotenc_callback)

    global lcd, backlight_timer
    lcd = LCD.Adafruit_CharLCD(**lcd_config)
    lcd.clear()
    lcd.set_backlight(0)
    backlight_timer = BackLightTimer(5, lcd)
    backlight_timer.start()

    global status, viewer
    viewer = MenuViewer(lcd, rotenc, status.menu)
    viewer.show()

    rospy.Subscriber("battery", BatteryVoltage, battery_callback)

    rospy.spin()


if __name__ == '__main__':
    main()
