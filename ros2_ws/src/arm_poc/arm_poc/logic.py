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


def clean_reply(line):
    """Strip line noise from a controller reply, keeping printable ASCII.

    With the servos on USB power, motion glitches the serial line and a real
    reply arrives as b'\\xff\\xff...OK'. Each glitch reads as a start bit
    followed by an idle-high line, which is exactly 0xFF. Treating that as a
    dead controller latched an unrecoverable fault on every move. Only
    non-printable bytes are dropped, so a genuine reset still fails: its boot
    chatter and READY survive and do not match the expected reply.
    """
    return bytes(b for b in line if 32 <= b < 127).strip()


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


def _solve3(a, b):
    """Gaussian elimination with partial pivoting for a 3x3 system."""
    m = [row[:] + [rhs] for row, rhs in zip(a, b)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-9:
            raise ValueError('Calibration points are collinear or coincident')
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for c in range(col, 4):
                m[r][c] -= f * m[col][c]
    return [m[i][3] / m[i][i] for i in range(3)]


def fit_pixel_to_joints(samples):
    """Least-squares affine map from image position to each joint angle.

    Each sample is (u, v, [four angles]) where u and v are the object's
    position in the frame, normalised to 0..1. Fits angle = a*u + b*v + c per
    joint, which assumes the objects lie roughly in a plane and the camera does
    not move. Both hold for a fixed-base arm watching a table; neither holds if
    the camera is mounted on the moving part.

    At least three non-collinear samples are required.
    """
    if len(samples) < 3:
        raise ValueError('Need at least three calibration points, got %d' % len(samples))
    for u, v, angles in samples:
        if not (0 <= u <= 1 and 0 <= v <= 1):
            raise ValueError('Calibration u and v must be normalised to 0..1')
        if len(angles) != 4:
            raise ValueError('Each calibration point needs exactly four angles')
    # Normal equations for [u, v, 1] against each joint.
    basis = [(u, v, 1.0) for u, v, _ in samples]
    ata = [[sum(p[i] * p[j] for p in basis) for j in range(3)] for i in range(3)]
    model = []
    for joint in range(4):
        atb = [sum(p[i] * s[2][joint] for p, s in zip(basis, samples)) for i in range(3)]
        model.append(tuple(_solve3([row[:] for row in ata], atb)))
    return model


def pose_from_pixel(model, u, v, lower, upper):
    """Joint angles for an object seen at (u, v), clamped to the safe limits.

    Clamping is deliberate: an extrapolated fit outside the calibrated area can
    ask for angles the mechanism cannot reach, and the limits are the last line
    of defence before the command is sent.
    """
    if len(model) != 4:
        raise ValueError('Model must cover four joints')
    pose = []
    for (a, b, c), lo, hi in zip(model, lower, upper):
        angle = int(round(a * u + b * v + c))
        pose.append(max(int(lo), min(int(hi), angle)))
    return pose
