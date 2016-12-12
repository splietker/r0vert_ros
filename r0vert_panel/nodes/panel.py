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


import rospy
from std_msgs.msg import String

from r0vert_panel import RotaryEncoder, EventType, Direction
from r0vert_panel.menu import Menu, MenuItem, MenuViewer

import Adafruit_CharLCD as LCD

# LED constants
lcd_rs = 23
lcd_en = 24
lcd_d4 = 25
lcd_d5 = 8
lcd_d6 = 7
lcd_d7 = 12
lcd_backlight = 18
lcd_columns = 16
lcd_rows = 2

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def rotation_callback(direction):
    print("rotation:", direction)

def button_callback(state):
    print("button:", state)

def main():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('panel', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    rotenc = RotaryEncoder(13, 19, 26)
    rotenc.register_callback(EventType.ROTATION, rotation_callback)
    rotenc.register_callback(EventType.BUTTON, button_callback)

    lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7,
                               lcd_columns, lcd_rows, lcd_backlight, enable_pwm=True)
    lcd.clear()
    lcd.message("Yay!")
    lcd.set_backlight(0)

    main_menu = Menu("main", True)
    main_menu.add(MenuItem("item 1", 4277789))
    main_menu.add(MenuItem("item 2", 13.3755))
    main_menu.add(MenuItem("item 3", True))
    main_menu.add(MenuItem("item 4 istolong", "meow"))
    viewer = MenuViewer(lcd, rotenc, main_menu)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    lcd.clear()
    lcd.set_backlight(1)

if __name__ == '__main__':
    main()
