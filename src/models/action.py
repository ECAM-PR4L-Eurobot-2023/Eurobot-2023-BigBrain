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

    def __str__(self):
        string = "--- Action ---\n"
        string += f"key: {self._key}\n"
        string += f"start coordinate: {self._start_coord}\n"
        string += f"displacement: {self._displacement}\n"

        return string