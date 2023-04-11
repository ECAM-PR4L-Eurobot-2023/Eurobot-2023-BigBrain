BACK_SWITCH_MASK = 0X01


class LimitSwitches:
    def __init__(self, value):
        self.value = value
    
    def is_back_pressed(self):
        return self.value & BACK_SWITCH_MASK