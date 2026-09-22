"""Interactive jog tool for the arm controller.

Moves one joint at a time in small steps so joint order and safe travel can be
found by hand. This is the tool for step 5 and 6 of docs/HARDWARE.md, and it
does the one thing a serial monitor cannot: it keeps resending the current
target, because the firmware detaches PWM two seconds after the last accepted
MOVE. Typing MOVE into a terminal makes the arm hold for two seconds and then
go limp.

    python tools/jog.py COM6                 # Windows
    python tools/jog.py /dev/ttyUSB0         # Linux
    python tools/jog.py COM6 --min 70 --max 110

Safety:
  * Requires CALIBRATED = true in the firmware. Until then every MOVE is
    refused and nothing here can move anything.
  * Starts at the current neutral pose and steps by one degree.
  * Clamps to --min/--max, which must be no wider than the firmware's own
    MIN_ANGLE/MAX_ANGLE or the controller replies ERR limits.
  * Space sends STOP, which releases holding torque. Support the arm first:
    an unpowered joint falls.
"""
import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial is required:  python -m pip install pyserial')

try:
    import msvcrt

    def get_key(timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if msvcrt.kbhit():
                return msvcrt.getch().decode('latin-1').lower()
            time.sleep(0.01)
        return None
except ImportError:
    import select
    import termios
    import tty

    def get_key(timeout):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1).lower()
        return None


NAMES = ['base (D27)', 'shoulder (D26)', 'elbow (D25)', 'gripper (D33)']
HELP = """
  1 2 3 4   select joint          + / -   step by one degree
  [ / ]     step by five          h       send all joints home
  space     STOP, release torque  q       quit (sends STOP)
"""


class Controller:
    def __init__(self, port, lower, upper, start):
        self.lower, self.upper = lower, upper
        self.pose = [start] * 4
        self.lock = threading.Lock()
        self.alive = True
        self.status = 'connecting'
        self.port = serial.Serial(port, 115200, timeout=0.4, write_timeout=0.4)
        time.sleep(2)               # Opening the port resets the ESP32.
        self.port.reset_input_buffer()
        if self.command(b'PING\n') != b'READY':
            raise SystemExit('Controller did not answer PING with READY')
        # Hold the pose continuously or the firmware drops PWM after 2 s.
        self.thread = threading.Thread(target=self.hold, daemon=True)
        self.thread.start()

    def command(self, line):
        with self.lock:
            self.port.reset_input_buffer()
            self.port.write(line)
            return self.port.readline().strip()

    def send_pose(self):
        with self.lock:
            pose = list(self.pose)
        reply = self.command(('MOVE %d %d %d %d\n' % tuple(pose)).encode('ascii'))
        self.status = reply.decode('ascii', 'replace') or 'no reply'
        return reply == b'OK'

    def hold(self):
        while self.alive:
            try:
                self.send_pose()
            except serial.SerialException as exc:
                self.status = 'serial error: %s' % exc
                self.alive = False
            time.sleep(0.4)

    def nudge(self, joint, delta):
        with self.lock:
            target = self.pose[joint] + delta
            self.pose[joint] = max(self.lower, min(self.upper, target))

    def close(self):
        self.alive = False
        time.sleep(0.5)
        try:
            self.command(b'STOP\n')
        finally:
            self.port.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('port', help='COM6, or /dev/serial/by-id/...')
    ap.add_argument('--min', type=int, default=80, dest='lower')
    ap.add_argument('--max', type=int, default=100, dest='upper')
    ap.add_argument('--start', type=int, default=90)
    args = ap.parse_args()
    if not 0 <= args.lower <= args.start <= args.upper <= 180:
        sys.exit('Require 0 <= min <= start <= max <= 180')

    arm = Controller(args.port, args.lower, args.upper, args.start)
    joint = 0
    print(__doc__.split('\n\n')[0])
    print('Limits %d to %d degrees. Support the arm before pressing space.' %
          (args.lower, args.upper))
    print(HELP)
    try:
        while arm.alive:
            with arm.lock:
                pose = list(arm.pose)
            print('\r  %-16s %s   [%s]      ' %
                  (NAMES[joint],
                   ' '.join(('>%3d<' if i == joint else ' %3d ') % a
                            for i, a in enumerate(pose)),
                   arm.status), end='', flush=True)
            key = get_key(0.15)
            if key is None:
                continue
            if key == 'q':
                break
            if key in '1234':
                joint = int(key) - 1
            elif key in '+=':
                arm.nudge(joint, 1)
            elif key == '-':
                arm.nudge(joint, -1)
            elif key == ']':
                arm.nudge(joint, 5)
            elif key == '[':
                arm.nudge(joint, -5)
            elif key == 'h':
                with arm.lock:
                    arm.pose = [args.start] * 4
            elif key == ' ':
                arm.alive = False
                arm.command(b'STOP\n')
                print('\nSTOP sent; holding torque released. Restart to continue.')
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        arm.close()
        print('Port closed, STOP sent.')


if __name__ == '__main__':
    main()
