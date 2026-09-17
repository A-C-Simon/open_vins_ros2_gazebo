#!/usr/bin/env python3
"""One-time global->odom alignment so RViz shows truth and VIO overlaid.

Wheel odometry lives in the `odom` frame, OpenVINS in its own `global`
frame. VIO global yaw is unobservable, so the two frames differ by a
fixed yaw (plus a small translation): paths have the right shape and
scale but look rotated apart, meeting only where both pass the origin.
This node measures that offset from the first meters both travel and
then publishes it as a latched static transform. Residual separation
after that is real estimator drift, which is exactly what you want to
see when judging VIO competence.

Method: record both start positions (for the translation part). For
the yaw part, use a window later in the run: headings come from
the displacement between the points where each stream first passes
win_start_m and win_end_m of its own travel (defaults 3 and 5 m).
Rationale: the first meters contain the drive ramp plus VIO filter
settling, so headings measured there are noisy; mature data agrees
to a couple of degrees. (A plain from-start threshold cannot exceed
the loop diameter on a looping path.) Translation still aligns the
two starts through that yaw: p_global = R(yaw) @ p_odom + t.
Publishes once via a static broadcaster, logs it, and exits.
If VINS never shows up (e.g. --no-vins) it publishes identity after
timeout_s so RViz still has a complete TF tree.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster


def norm_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Aligner(Node):
    def __init__(self):
        super().__init__('align_frames')
        self.declare_parameter('win_start_m', 3.0)
        self.declare_parameter('win_end_m', 5.0)
        self.declare_parameter('timeout_s', 180.0)
        self.odom0 = None
        self.vio0 = None
        self.odom_now = None
        self.vio_now = None
        # latched path length per stream plus window edge snapshots
        self.odom_dist = 0.0
        self.vio_dist = 0.0
        self.odom_prev = None
        self.vio_prev = None
        self.odom_ws = None
        self.odom_we = None
        self.vio_ws = None
        self.vio_we = None
        self.done = False
        self.t_start = self.get_clock().now()
        self.br = StaticTransformBroadcaster(self)
        # Provisional identity so the TF tree (and RViz) is complete from
        # second zero; replaced by the measured alignment once available.
        self.publish_tf(0.0, 0.0, 0.0, 0.0, 'provisional identity until measured')
        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/ov_msckf/poseimu', self.on_vio, 20)
        self.create_timer(0.5, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y, p.z)
        if self.odom0 is None:
            self.odom0 = xy
            self.odom_prev = xy
        self.odom_dist += math.hypot(xy[0] - self.odom_prev[0], xy[1] - self.odom_prev[1])
        self.odom_prev = xy
        self.odom_now = xy
        ws = self.get_parameter('win_start_m').value
        we = self.get_parameter('win_end_m').value
        if self.odom_ws is None and self.odom_dist >= ws:
            self.odom_ws = xy
        if self.odom_we is None and self.odom_dist >= we:
            self.odom_we = xy

    def on_vio(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y, p.z)
        if self.vio0 is None:
            self.vio0 = xy
            self.vio_prev = xy
        self.vio_dist += math.hypot(xy[0] - self.vio_prev[0], xy[1] - self.vio_prev[1])
        self.vio_prev = xy
        self.vio_now = xy
        ws = self.get_parameter('win_start_m').value
        we = self.get_parameter('win_end_m').value
        if self.vio_ws is None and self.vio_dist >= ws:
            self.vio_ws = xy
        if self.vio_we is None and self.vio_dist >= we:
            self.vio_we = xy

    def publish_tf(self, tx, ty, tz, yaw, why):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'global'
        t.child_frame_id = 'odom'
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(t)
        self.get_logger().info('global->odom: t=(%.2f,%.2f,%.2f) yaw=%.1fdeg (%s)'
                               % (tx, ty, tz, math.degrees(yaw), why))

    def tick(self):
        if self.done:
            return
        now = self.get_clock().now()
        if (now - self.t_start).nanoseconds / 1e9 > self.get_parameter('timeout_s').value:
            self.publish_tf(0.0, 0.0, 0.0, 0.0, 'timeout fallback, VINS unseen')
            self.finish()
            return
        if self.odom_we is None or self.vio_we is None:
            return
        dox = self.odom_we[0] - self.odom_ws[0]
        doy = self.odom_we[1] - self.odom_ws[1]
        dvx = self.vio_we[0] - self.vio_ws[0]
        dvy = self.vio_we[1] - self.vio_ws[1]
        yaw_off = norm_angle(math.atan2(dvy, dvx) - math.atan2(doy, dox))
        c, s = math.cos(yaw_off), math.sin(yaw_off)
        tx = self.vio0[0] - (c * self.odom0[0] - s * self.odom0[1])
        ty = self.vio0[1] - (s * self.odom0[0] + c * self.odom0[1])
        tz = self.vio0[2] - self.odom0[2]
        self.publish_tf(tx, ty, tz, yaw_off, 'window %.0f-%.0fm' % (
            self.get_parameter('win_start_m').value, self.get_parameter('win_end_m').value))
        self.finish()

    def finish(self):
        self.done = True
        # latched static transform stays alive; give it a moment on the wire
        import time
        t0 = time.time()
        while time.time() - t0 < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(Aligner())
    except SystemExit:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
