"""Nonblocking, resumable movement routines. Speeds are -1 to +1."""
# Each step: (left speed, right speed, seconds of permitted movement).
FIGURE_EIGHT = [(0.28, 0.48, 4.0), (0.48, 0.28, 4.0)]
REVERSE_AND_TURN = [(-0.35, -0.35, 1.5), (0.35, -0.35, 0.8),
                    (0.35, 0.35, 1.5)]


class Maneuver:
    def __init__(self):
        self.cancel()

    @property
    def active(self):
        return self.steps is not None

    def start(self, steps):
        self.steps = steps
        self.index = 0
        self.elapsed = 0.0

    def cancel(self):
        self.steps = None
        self.index = 0
        self.elapsed = 0.0

    def command(self):
        if not self.active:
            return 0.0, 0.0
        left, right, _ = self.steps[self.index]
        return left, right

    def advance(self, dt):
        # Only call for time during which the motors were permitted to move.
        if not self.active:
            return
        self.elapsed += dt
        if self.elapsed >= self.steps[self.index][2]:
            self.index += 1
            self.elapsed = 0.0
            if self.index >= len(self.steps):
                self.cancel()
