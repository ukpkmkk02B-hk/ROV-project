from collections import deque


class MovingAverage:
    def __init__(self, window_size: int):
        self.values = deque(maxlen=max(1, int(window_size)))

    def update(self, value: float) -> float:
        self.values.append(float(value))
        return sum(self.values) / len(self.values)

    def reset(self) -> None:
        self.values.clear()
