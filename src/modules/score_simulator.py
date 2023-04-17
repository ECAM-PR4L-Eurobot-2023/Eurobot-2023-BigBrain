SCORE_OFFSET = 25


class ScoreSimulator:
    def __init__(self):
        self._score_offset = SCORE_OFFSET
        self._score_cherry = 0
    
    @property
    def score(self):
        print(self._score_cherry)
        total_score = self._score_offset + self._score_cherry

        return total_score if self._score_cherry == 0 else total_score + 5

    @property
    def score_offset(self):
        return self._score_offset

    @property
    def score_cherry(self):
        return self._score_cherry

    @score_cherry.setter
    def score_cherry(self, new_score):
        self._score_cherry = new_score

    def reset(self):
        self._score_cherry = 0
