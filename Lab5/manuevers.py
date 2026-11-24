"""Nonblocking, resumable movement routines. Speeds are -1 to +1."""
# Each step: (left speed, right speed, seconds of permitted movement).
# Approximate a figure eight using two opposite curved driving segments.
FIGURE_EIGHT = [(0.28, 0.48, 4.0), (0.48, 0.28, 4.0)]
# Reverse straight, pivot in place, then drive forward.
REVERSE_AND_TURN = [(-0.35, -0.35, 1.5), (0.35, -0.35, 0.8),
                    (0.35, 0.35, 1.5)]


# Track one routine without blocking the main sensor and controller loop.
class Maneuver:
    # Begin with no active maneuver.
    def __init__(self):
        self.cancel()

    # Expose whether a routine is active as a read-only property.
    @property
    def active(self):
        return self.steps is not None

    # Load a sequence of steps and reset progress to the beginning.
    def start(self, steps):
        self.steps = steps
        self.index = 0
        self.elapsed = 0.0

    # Discard the current routine and reset its timing state.
    def cancel(self):
        self.steps = None
        self.index = 0
        self.elapsed = 0.0

    # Return the current step speeds, or zero speeds when idle.
    def command(self):
        if not self.active:
            return 0.0, 0.0
        # Ignore the duration here; advance() handles step timing separately.
        left, right, _ = self.steps[self.index]
        return left, right

    # Accumulate permitted movement time rather than total wall-clock time.
    def advance(self, dt):
        # Only call for time during which the motors were permitted to move.
        if not self.active:
            return
        # Keep the current step until its configured driving duration has elapsed.
        self.elapsed += dt
        if self.elapsed >= self.steps[self.index][2]:
            # Move to the next step and reset the elapsed time.
            self.index += 1
            self.elapsed = 0.0
            # Finish the routine when all steps have been completed.
            if self.index >= len(self.steps):
                self.cancel()
