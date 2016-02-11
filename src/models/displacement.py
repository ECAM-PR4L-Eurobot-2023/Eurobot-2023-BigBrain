class Displacement:
    def __init__(self, angle_start, angle_end, distance):
        self.angle_start = angle_start
        self.angle_end = angle_end
        self.distance = distance

    def __str__(self):
        string = "--- Displacement ---\n"
        string += "angle start: {}\n".format(self.angle_start)
        string += "angle end: {}\n".format(self.angle_end)
        string += "distance: {}\n".format(self.distance)

        return string