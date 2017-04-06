import glob
from r0vert_panel.menu import Menu, MenuItem


class BatteryMenuItem(MenuItem):
    def value_string(self):
        return "%.2fV" % self.value


class Status(Menu):
    def __init__(self):
        super(Status, self).__init__("status", True)

        self.battery1 = BatteryMenuItem("Bat1", 0.0)
        self.battery2 = BatteryMenuItem("Bat2", 0.0)

        self.joy = MenuItem("Joy", False)

        self.add(self.battery1)
        self.add(self.battery2)
        self.add(self.joy)

    def update(self):
        if len(glob.glob("/sys/class/power_supply/sony_controller_battery*")) > 0:
            self.joy.value = True
        else:
            self.joy.value = False
