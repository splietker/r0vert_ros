from r0vert_panel.menu import Menu, MenuItem


class BatteryMenuItem(MenuItem):
    def value_string(self):
        return "%.2fV" % self.value


class Status:
    def __init__(self):
        self.battery1 = BatteryMenuItem("Bat1", 0.0)
        self.battery2 = BatteryMenuItem("Bat2", 0.0)

        self.menu = Menu("status", True)
        self.menu.add(self.battery1)
        self.menu.add(self.battery2)
