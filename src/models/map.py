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
