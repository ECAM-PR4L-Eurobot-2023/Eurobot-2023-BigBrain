TOLERANCE = 10.0  # mm


class Map:
    def __init__(self, width, length, plates, cherries, aruco_tags):
        self._width = width
        self._length = length
        self._plates = plates
        self._cherries = cherries
        self._aruco_tags = aruco_tags

    @property 
    def width(self):
        return self._width

    @property
    def length(self):
        return self._length

    @property
    def plates(self):
        return self._plates

    @property
    def cherries(self):
        return self._cherries

    @property
    def aruco_tags(self):
        return self._aruco_tags

    def is_wall(self, x, y):
        return (not -TOLERANCE <= x <= self.width) or (not -TOLERANCE <= y <= self.length)

    def is_cherry(self, x, y):
        for cherry, cherry_dict in self._cherries.items():
            x_mid_size = cherry_dict['x_size'] / 2
            y_mid_size = cherry_dict['y_size'] / 2
            min_x_bound = cherry_dict['x_pos'] - x_mid_size - TOLERANCE 
            max_x_bound = cherry_dict['x_pos'] + x_mid_size + TOLERANCE
            min_y_bound = cherry_dict['y_pos'] - y_mid_size - TOLERANCE
            max_y_bound = cherry_dict['y_pos'] + y_mid_size + TOLERANCE

            # Check if the values are in the place of cherries
            if min_x_bound <= x <= max_x_bound and min_y_bound <= y <= max_y_bound:
                return True

        return False
