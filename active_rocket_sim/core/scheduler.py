from __future__ import annotations


class RateGate:
    def __init__(self, rate_hz: float):
        self.period = 1.0 / rate_hz
        self._next_t = 0.0

    def due(self, t: float) -> bool:
        if t + 1e-12 >= self._next_t:
            self._next_t += self.period
            return True
        return False
