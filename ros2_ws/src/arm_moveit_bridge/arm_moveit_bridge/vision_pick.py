"""Turn a detected object into a MoveIt goal above it.

arm_poc already finds the object: it publishes /vision/target_point as the
blob centre in normalised image coordinates (x = u, y = v, both 0..1, and
z = the fraction of the frame the blob fills). What it does not know is where
that is in space, and neither does MoveIt until someone says where the camera
is looking from.

This node closes that gap the only way a single camera can: it assumes the
object is lying on a known plane. A pixel gives a ray, the table gives a
plane, and the intersection gives a point. That is enough to plan to, and it
is wrong the moment the object is not on the table -- held up, stacked on a
box, or seen from an angle the camera pose does not describe.

EVERY NUMBER BELOW IS UNVERIFIED. The camera pose, the field of view and the
table height are declared parameters with placeholder defaults, because
nothing in this project has measured any of them. Until they are measured this
node computes a confident answer to the wrong question, so it plans and does
not execute: plan_only defaults to true.

Reach is also checked before planning. With the placeholder link lengths the
arm can only reach a shell roughly 0.16-0.22 m from the shoulder, so a target
on the near side of the table is unreachable no matter how well it was seen.
Saying so here is more useful than an OMPL sampling failure.
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose, PointStamped, TransformStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, MotionPlanRequest, PositionConstraint,
                             PlanningOptions)
from shape_msgs.msg import SolidPrimitive
from tf2_ros import StaticTransformBroadcaster

GROUP = 'arm'
TOOL = 'tool0'
BASE = 'base_link'


def rpy_matrix(roll, pitch, yaw):
    """Rotation matrix, ZYX convention, as three row tuples."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class VisionPick(Node):

    def __init__(self):
        super().__init__('arm_moveit_pick')

        # -- UNVERIFIED geometry. Measure these before trusting a result. ----
        # Camera behind the arm, looking down and forward across the table.
        # From (0, -0.35, 0.30) a roll of -2.078 rad puts the optical axis on
        # the table 0.19 m in front of the base, so the centre of the frame is
        # inside the reachable ring. Self-consistent, which is not the same as
        # correct. Optical convention: z along the view, x right, y down.
        self.declare_parameter('camera_xyz', [0.0, -0.35, 0.30])
        self.declare_parameter('camera_rpy', [-2.078, 0.0, 0.0])
        self.declare_parameter('camera_hfov_deg', 65.0)
        self.declare_parameter('image_aspect', 4.0 / 3.0)
        self.declare_parameter('table_z', 0.0)
        # --------------------------------------------------------------------

        self.declare_parameter('approach_height', 0.05)
        self.declare_parameter('tolerance', 0.015)
        # Measured from the shoulder pivot, not the base plate: that is where
        # the two-link chain actually starts. With placeholder link lengths the
        # elbow limit puts the near edge at 0.16 m.
        self.declare_parameter('shoulder_z', 0.067)
        self.declare_parameter('min_radius', 0.16)
        self.declare_parameter('max_radius', 0.22)
        # A bounding box around a cup at arm's length fills one to three
        # percent of a 320x240 frame. The colour detector's 0.05 was a blob
        # threshold and rejects every real object here.
        self.declare_parameter('min_fraction', 0.005)
        self.declare_parameter('cooldown_seconds', 5.0)
        self.declare_parameter('plan_only', True)
        # Either detector publishes the same message shape: arm_poc's colour
        # threshold on /vision/target_point, arm_vision's neural detector on
        # /vision/object_point. Switch source without touching this node.
        self.declare_parameter('target_topic', '/vision/target_point')

        g = lambda n: self.get_parameter(n).value
        self.cam_xyz = list(g('camera_xyz'))
        self.cam_rpy = list(g('camera_rpy'))
        self.hfov = math.radians(g('camera_hfov_deg'))
        self.aspect = g('image_aspect')
        self.table_z = g('table_z')
        self.approach = g('approach_height')
        self.tolerance = g('tolerance')
        self.shoulder_z = g('shoulder_z')
        self.min_r, self.max_r = g('min_radius'), g('max_radius')
        self.min_fraction = g('min_fraction')
        self.cooldown = g('cooldown_seconds')
        self.plan_only = g('plan_only')
        self.topic = g('target_topic')

        self.rot = rpy_matrix(*self.cam_rpy)
        self.last_goal = 0.0
        self.busy = False

        self.static_tf()

        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.create_subscription(
            PointStamped, self.topic, self.on_target, 10)

        mode = 'PLAN ONLY, nothing will move' if self.plan_only else \
               'EXECUTE, a detection will move the arm'
        self.get_logger().info(
            f'Vision pick ready ({mode}), watching {self.topic}.')
        self.get_logger().warn(
            'Camera pose, field of view and table height are unmeasured '
            'placeholders. Targets are only as right as those numbers.')

    def static_tf(self):
        """Publish the assumed camera pose so RViz shows what is assumed."""
        self.tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = BASE
        t.child_frame_id = 'camera_optical_frame'
        t.transform.translation.x = float(self.cam_xyz[0])
        t.transform.translation.y = float(self.cam_xyz[1])
        t.transform.translation.z = float(self.cam_xyz[2])
        q = quat_from_rpy(*self.cam_rpy)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        self.tf.sendTransform(t)

    # -- pixel to point ------------------------------------------------------

    def ray(self, u, v):
        """Unit ray through a normalised pixel, in base_link.

        Optical convention: z forward along the view, x right, y down. The
        vertical field of view follows from the horizontal one and the aspect
        ratio, which is what a pinhole with square pixels gives.
        """
        tx = math.tan(self.hfov / 2.0)
        ty = tx / self.aspect
        d_cam = ((u - 0.5) * 2.0 * tx, (v - 0.5) * 2.0 * ty, 1.0)
        d = [sum(self.rot[i][k] * d_cam[k] for k in range(3)) for i in range(3)]
        n = math.sqrt(sum(c * c for c in d)) or 1.0
        return [c / n for c in d]

    def ground(self, u, v):
        """Where that ray meets the table. None if it never does."""
        d = self.ray(u, v)
        if abs(d[2]) < 1e-6 or (self.table_z - self.cam_xyz[2]) / d[2] <= 0:
            return None                      # parallel to the table, or behind
        s = (self.table_z - self.cam_xyz[2]) / d[2]
        return [self.cam_xyz[i] + s * d[i] for i in range(3)]

    # -- planning ------------------------------------------------------------

    def on_target(self, message):
        u, v, fraction = message.point.x, message.point.y, message.point.z
        now = time.monotonic()
        if self.busy or now - self.last_goal < self.cooldown:
            return
        if fraction < self.min_fraction:
            self.last_goal = now
            self.get_logger().info(
                f'Ignoring a detection filling {fraction:.3f} of the frame, '
                f'below min_fraction {self.min_fraction}')
            return

        point = self.ground(u, v)
        if point is None:
            self.last_goal = now
            self.get_logger().warn(
                f'Pixel ({u:.2f}, {v:.2f}) does not meet the table plane; '
                'check camera_rpy')
            return

        point[2] += self.approach
        radius = math.sqrt(point[0] ** 2 + point[1] ** 2
                           + (point[2] - self.shoulder_z) ** 2)
        if not self.min_r <= radius <= self.max_r:
            self.last_goal = now
            self.get_logger().warn(
                f'Object at ({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) '
                f'is {radius:.3f} m away, outside the {self.min_r:.2f}-'
                f'{self.max_r:.2f} m the arm can reach')
            return

        self.last_goal = now
        self.busy = True
        self.get_logger().info(
            f'Object at pixel ({u:.2f}, {v:.2f}) -> approach '
            f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})')
        self.send(point)

    def send(self, point):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('move_action not available; is move_group up?')
            self.busy = False
            return

        pc = PositionConstraint()
        pc.header.frame_id = BASE
        pc.link_name = TOOL
        pc.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE,
                           dimensions=[self.tolerance]))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point
        pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        request = MotionPlanRequest(
            group_name=GROUP, num_planning_attempts=5,
            allowed_planning_time=5.0,
            max_velocity_scaling_factor=1.0,
            max_acceleration_scaling_factor=1.0)
        request.goal_constraints.append(
            Constraints(position_constraints=[pc]))

        goal = MoveGroup.Goal(request=request,
                              planning_options=PlanningOptions(
                                  plan_only=self.plan_only))
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('move_group rejected the goal')
            self.busy = False
            return
        handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        result = future.result().result
        code = result.error_code.val
        points = len(result.planned_trajectory.joint_trajectory.points)
        if code == 1:
            self.get_logger().info(
                f'{"Planned" if self.plan_only else "Executed"}: '
                f'{points} trajectory points')
        else:
            self.get_logger().warn(f'Planning failed, error_code={code}')
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = VisionPick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
