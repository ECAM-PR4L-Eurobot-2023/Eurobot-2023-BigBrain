class LidarBottom:
    def __init__(self):
        self._distances = []
        self._angle_precision = 0.0
        self._is_new_read = False

    @property
    def distances(self):
        return tuple(self._distances)
    
    @property
    def angle_precision(self):
        return self._angle_precision

    @property
    def is_new_read(self):
        if not self._is_new_read:
            return False

        self._is_new_read = False
        return True

    def set_distances(self, distances, angle_precision):
        self._distances = distances[:]
        self._angle_precision = angle_precision
        self._is_new_read = True



