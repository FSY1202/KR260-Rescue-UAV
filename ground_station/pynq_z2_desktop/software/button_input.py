"""Time based debounce: emit one event per released-to-pressed BTN0 edge."""


class ButtonPress:
    def __init__(self, initial_level, now, debounce_seconds=0.04):
        self.candidate = bool(initial_level)
        self.stable = self.candidate
        self.since = now
        self.interval = debounce_seconds

    def update(self, level, now):
        level = bool(level)
        if level != self.candidate:
            self.candidate = level
            self.since = now
        if self.candidate != self.stable and now-self.since >= self.interval:
            self.stable = self.candidate
            return self.stable
        return False
