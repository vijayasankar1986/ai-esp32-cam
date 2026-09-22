import math


def move_command(pose, lower, upper):
    if not (len(pose) == len(lower) == len(upper) == 4):
        raise ValueError('Exactly four joint values required')
    for angle, lo, hi in zip(pose, lower, upper):
        if not all(math.isfinite(x) for x in (angle, lo, hi)):
            raise ValueError('Non-finite angle')
        if not (0 <= lo <= angle <= hi <= 180) or int(angle) != angle:
            raise ValueError('Invalid angle or limits')
    return ('MOVE ' + ' '.join(str(int(x)) for x in pose) + '\n').encode('ascii')


class DetectionGate:
    def __init__(self, frames):
        if frames < 1:
            raise ValueError('frames must be positive')
        self.frames = frames
        self.positive = self.negative = 0
        self.armed = True

    def update(self, detected):
        self.positive = self.positive + 1 if detected else 0
        self.negative = 0 if detected else self.negative + 1
        if self.negative >= self.frames:
            self.armed = True
        if self.armed and self.positive >= self.frames:
            self.armed = False
            return True
        return False
