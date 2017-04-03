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


from threading import Timer, Event

from rotary_encoder import EventType, Direction


class MenuItem:
    def __init__(self, name, value=None):
        self.name = name
        self.value = value

    def value_string(self):
        return str(self.value)


class Menu(MenuItem):
    def __init__(self, name, is_main_menu=False):
        # super(Menu, self).__init__(name)
        self.name = name
        self.items = list()

        self.is_main_menu = is_main_menu

    def add(self, menu_item):
        self.items.append(menu_item)


class MenuViewer:
    def __init__(self, lcd, rotary_encoder, menu, scroll_overflow=False):
        self.__lcd = lcd
        self.__menu = menu
        self.__current_menu = self.__menu
        self.__scroll_overflow = scroll_overflow

        self.__selected_item = 0
        self.__selected_row = 0

        self.__max_name_length = 8
        self.__max_value_length = 6

        self.__name_scroll_index = 0
        self.__name_scroll_direction = 1
        self.__name_scroll_skip = 3
        self.__value_scroll_index = 0
        self.__value_scroll_direction = 1
        self.__value_scroll_skip = 3

        self.__rotary_encoder = rotary_encoder
        self.__rotary_encoder.register_callback(EventType.ROTATION, self.__rotary_encoder_rotation)
        self.__rotary_encoder.register_callback(EventType.BUTTON, self.__rotary_encoder_button)

        self.__stopped = False

        Timer(1.0, self.scroll).start()

    def stop(self):
        self.__stopped = True

    def __rotary_encoder_rotation(self, direction):
        prior_item = self.__selected_item
        if len(self.__current_menu.items) == 0:
            return
        self.__selected_row += direction
        self.__selected_row = min(self.__selected_row, self.__lcd._lines - 1)
        self.__selected_row = max(self.__selected_row, 0)

        self.__selected_item += direction
        if self.__scroll_overflow:
            if self.__selected_item >= len(self.__current_menu.items):
                self.__selected_item = 0
                self.__selected_row = 0
            elif self.__selected_item < 0:
                self.__selected_item = len(self.__current_menu.items) - 1
                self.__selected_row = self.__lcd._lines - 1
        else:
            self.__selected_item = min(self.__selected_item, len(self.__current_menu.items) - 1)
            self.__selected_item = max(self.__selected_item, 0)

        if prior_item != self.__selected_item:
            # Reset scrolling
            self.__name_scroll_index = 0
            self.__name_scroll_direction = 1
            self.__name_scroll_skip = 3
            self.__value_scroll_index = 0
            self.__value_scroll_direction = 1
            self.__value_scroll_skip = 3

            self.show()

    def __rotary_encoder_button(self, state):
        self.show()

    def scroll(self):
        item = self.__menu.items[self.__selected_item]
        name_length = len(item.name)
        if self.__name_scroll_skip > 0:
            self.__name_scroll_skip -= 1
        elif name_length > self.__max_name_length:
            self.__name_scroll_index += self.__name_scroll_direction
            if self.__max_name_length + self.__name_scroll_index >= name_length or self.__name_scroll_index == 0:
                self.__name_scroll_direction *= -1
                self.__name_scroll_skip = 3
            name = item.name[self.__name_scroll_index:self.__max_name_length + self.__name_scroll_index]
            self.__lcd.set_cursor(1, self.__selected_row)
            self.__lcd.message(name)

        value_length = len(item.value_string())
        if self.__value_scroll_skip > 0:
            self.__value_scroll_skip -= 1
        elif value_length > self.__max_value_length:
            self.__value_scroll_index += self.__value_scroll_direction
            if self.__max_value_length + self.__value_scroll_index >= value_length or self.__value_scroll_index == 0:
                self.__value_scroll_direction *= -1
                self.__value_scroll_skip = 3
            value = item.value_string()[self.__value_scroll_index:self.__max_value_length + self.__value_scroll_index]
            self.__lcd.set_cursor(self.__lcd._cols - self.__max_value_length, self.__selected_row)
            self.__lcd.message(value)

        if not self.__stopped:
            Timer(0.5, self.scroll).start()

    def show(self):
        self.__lcd.clear()
        lower_index = self.__selected_item - self.__selected_row
        row = 0
        for index in range(lower_index, lower_index + self.__lcd._lines):
            item = self.__menu.items[index]
            if row == self.__selected_row:
                self.__lcd.message("~")  # ~ equals an arrow
            else:
                self.__lcd.message(" ")

            self.__lcd.message(item.name[0:self.__max_name_length])

            value_string = str(item.value_string())
            value_length = min(len(value_string), self.__max_value_length)
            self.__lcd.set_cursor(self.__lcd._cols - value_length, row)
            self.__lcd.message(value_string)

            self.__lcd.message("\n")
            row += 1
