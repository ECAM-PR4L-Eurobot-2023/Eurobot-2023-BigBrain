class Action:
    def __init__(self, key, start_coord, end_coord, displacement):
        self._key = key
        self._start_coord = start_coord
        self._end_coord = end_coord
        self._displacement = displacement

    @property
    def key(self):
        return self._key

    @property
    def start_coord(self):
        return self._start_coord

    @property
    def end_coord(self):
        return self._end_coord

    @property
    def displacement(self):
        return self._displacement