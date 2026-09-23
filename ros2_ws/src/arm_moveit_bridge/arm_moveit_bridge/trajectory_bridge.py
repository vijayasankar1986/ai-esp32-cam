"""Execute MoveIt trajectories on the arm without owning the serial port.

Only one process may hold the USB serial port; arm_poc is that process, and
taking it away from the node would cost the camera watchdog, the joint limits
and the manual-pose timeout along with it. So this bridge never opens serial.
It accepts FollowJointTrajectory goals from MoveIt and republishes them as
ordinary manual poses on /arm/manual_pose, which arm_poc already validates
against min_angles/max_angles before anything reaches a servo.

Two conversions live here and nowhere else:

  Units. arm_poc speaks the servo scale, where 90 degrees is the neutral pose
  it homes to, and puts that number through math.radians before publishing. The
  URDF is zero-centred like any other MoveIt model. OFFSET is the difference.

  Time. A planned trajectory assumes every joint moves at once. This firmware
  steps one joint at a time under SEQUENTIAL_MOTION and ramps about a degree
  every 20 ms, so the arm is still moving long after the trajectory's nominal
  end. The bridge holds the final pose and waits out an estimate of the real
  motion before reporting success, and keeps republishing throughout: a manual
  pose that stops being refreshed lapses after manual_timeout and the arm
  returns home, which mid-trajectory would look like the arm fighting back.

What this cannot do: verify anything. There is no position feedback on this
build. Success here means the poses were accepted by the node, not that the
arm reached them.
"""

import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

JOINTS = ['joint0', 'joint1', 'joint2', 'joint3']

#: Servo neutral. The firmware homes to 90 degrees on every channel, and that
#: pose is zero in the URDF.
OFFSET = math.pi / 2

#: Firmware ramp, seconds per degree, from docs/HARDWARE.md.
RAMP_S_PER_DEG = 0.02

#: Joints the firmware drives one after another, not together.
SEQUENTIAL = 4


class TrajectoryBridge(Node):

    def __init__(self):
        super().__init__('arm_moveit_bridge')

        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('settle_margin', 1.5)
        self.declare_parameter('max_settle_seconds', 30.0)

        self.hz = self.get_parameter('publish_hz').value
        self.margin = self.get_parameter('settle_margin').value
        self.max_settle = self.get_parameter('max_settle_seconds').value

        self.manual = self.create_publisher(JointState, '/arm/manual_pose', 10)
        self.states = self.create_publisher(JointState, '/joint_states', 10)

        # arm_poc publishes the commanded pose on its own topic in servo units.
        # MoveIt wants /joint_states in model units, so translate rather than
        # ask either side to change.
        self.create_subscription(
            JointState, '/arm/joint_states', self.on_arm_state, 10)

        self.last_pose = [0.0] * len(JOINTS)
        self.executing = False

        # Reentrant so a cancel request is still served while a goal is being
        # executed; the execute callback blocks for the length of the motion.
        self.server = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            'MoveIt bridge ready. Trajectories are published to '
            '/arm/manual_pose; arm_poc keeps the serial port and the limits.')

    # -- state ---------------------------------------------------------------

    def on_arm_state(self, message):
        """Republish arm_poc's commanded pose as /joint_states, model units."""
        if len(message.position) != len(JOINTS):
            return
        self.last_pose = [p - OFFSET for p in message.position]
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(JOINTS)
        out.position = list(self.last_pose)
        self.states.publish(out)

    def send(self, pose):
        """Publish one absolute pose in servo units.

        Absolute, never incremental: a dropped message then costs one update
        instead of accumulating into motion nobody asked for.
        """
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINTS)
        message.position = [p + OFFSET for p in pose]
        self.manual.publish(message)

    # -- action --------------------------------------------------------------

    def on_goal(self, goal):
        if self.executing:
            self.get_logger().warn('Rejected: a trajectory is already running')
            return GoalResponse.REJECT
        names = list(goal.trajectory.joint_names)
        if not names or not set(names) <= set(JOINTS):
            self.get_logger().warn(f'Rejected: unexpected joints {names}')
            return GoalResponse.REJECT
        if not goal.trajectory.points:
            self.get_logger().warn('Rejected: trajectory has no points')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal):
        # Stopping means stopping where it is: hold the pose reached so far.
        # Not refreshing instead would let it lapse and send the arm home,
        # which is movement, and cancelling should not cause movement.
        return CancelResponse.ACCEPT

    def execute(self, goal):
        self.executing = True
        try:
            return self.run(goal)
        finally:
            self.executing = False

    def run(self, goal):
        trajectory = goal.request.trajectory
        names = list(trajectory.joint_names)
        index = [names.index(j) if j in names else None for j in JOINTS]
        result = FollowJointTrajectory.Result()

        points = trajectory.points
        start = self.get_clock().now()
        period = 1.0 / self.hz
        held = list(self.last_pose)

        def pose_at(point):
            """Full four-joint pose: joints the goal omits keep their value."""
            pose = list(held)
            for j, i in enumerate(index):
                if i is not None and i < len(point.positions):
                    pose[j] = point.positions[i]
            return pose

        # Walk the trajectory in wall time, sending whichever point is current.
        # MoveIt has already spaced the points; re-interpolating between them
        # would add nothing the firmware's own ramp does not do.
        cursor = 0
        while cursor < len(points):
            if goal.is_cancel_requested:
                self.send(pose_at(points[max(cursor - 1, 0)]))
                goal.canceled()
                self.get_logger().info('Trajectory cancelled, holding pose')
                result.error_code = result.SUCCESSFUL
                return result

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            while (cursor + 1 < len(points)
                   and self.stamp(points[cursor + 1]) <= elapsed):
                cursor += 1

            pose = pose_at(points[cursor])
            self.send(pose)

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = list(JOINTS)
            feedback.desired.positions = list(pose)
            feedback.actual.positions = list(self.last_pose)
            goal.publish_feedback(feedback)

            if cursor + 1 >= len(points) and elapsed >= self.stamp(points[-1]):
                break
            time.sleep(period)

        final = pose_at(points[-1])
        self.settle(goal, final, held)

        result.error_code = result.SUCCESSFUL
        result.error_string = (
            'Poses accepted by arm_poc. This build has no position feedback, '
            'so the arm is not confirmed to have reached them.')
        goal.succeed()
        return result

    @staticmethod
    def stamp(point):
        return Duration.from_msg(point.time_from_start).nanoseconds / 1e9

    def settle(self, goal, final, before):
        """Hold the final pose while the hardware catches up with the plan.

        The estimate is the whole travel at the firmware's ramp rate, times the
        number of joints it steps through one at a time, times a margin. It is
        an upper bound on a machine that cannot be asked how far it has got.
        """
        degrees = sum(abs(math.degrees(a - b)) for a, b in zip(final, before))
        wait = min(degrees * RAMP_S_PER_DEG * SEQUENTIAL * self.margin,
                   self.max_settle)
        if wait <= 0:
            return
        self.get_logger().info(f'Holding final pose for {wait:.1f}s to settle')
        until = time.monotonic() + wait
        while time.monotonic() < until:
            if goal.is_cancel_requested:
                return
            self.send(final)
            time.sleep(1.0 / self.hz)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
