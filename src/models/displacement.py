class Displacement:
    def __init__(self, angle_start, angle_end, x, y):
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.x = x
        self.y = y

    def __str__(self):
        string = "--- Displacement ---\n"
        string += "angle start: {}\n".format(self.angle_start)
        string += "angle end: {}\n".format(self.angle_end)
        string += "x: {}\n".format(self.x)
        string += "y: {}\n".format(self.y)

        return string