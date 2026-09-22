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


# HSV ranges for the colours the vision node can target. OpenCV hue is 0-179,
# so red wraps the end of the circle and needs two ranges. Saturation and
# value floors reject grey and near-black pixels that would otherwise match
# any hue under poor lighting.
COLOR_RANGES = {
    'red': (((0, 100, 70), (10, 255, 255)), ((170, 100, 70), (179, 255, 255))),
    'green': (((36, 80, 60), (85, 255, 255)),),
    'blue': (((90, 80, 60), (130, 255, 255)),),
    'yellow': (((20, 100, 80), (35, 255, 255)),),
    'orange': (((11, 120, 90), (19, 255, 255)),),
    'purple': (((131, 60, 60), (160, 255, 255)),),
}


def color_ranges(name):
    """HSV inRange bounds for a named colour, validated."""
    if not isinstance(name, str):
        raise ValueError('Colour must be a string')
    key = name.strip().lower()
    if key not in COLOR_RANGES:
        raise ValueError(
            'Unknown colour %r; choose one of %s' % (name, ', '.join(sorted(COLOR_RANGES))))
    return COLOR_RANGES[key]
