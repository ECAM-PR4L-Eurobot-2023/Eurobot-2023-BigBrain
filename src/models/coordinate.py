class Coordinate:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def __str__(self):
        string = "--- Coordinate ---\n"
        string += "x: {}\n".format(self.x)
        string += "y: {}\n".format(self.y)
        string += "angle: {}\n".format(self.angle)

        return string

    def __eq__(self, other):
        return (self.x == other.x) and (self.y == other.y)