#!/usr/bin/env python3
"""Drive a square loop on /cmd_vel while enabled.
Exclusive ownership: only ONE node may publish /cmd_vel (a second writer,
even an idle keyboard node spamming zeros, wins intermittently and the
rover stutters or freezes). Run either auto_loop OR key_teleop, never both
(gazebo_test.sh enforces this with exclusive --auto / --teleop modes).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoLoop(Node):
    def __init__(self):
        super().__init__('auto_loop')
        # Declared WITHOUT a preset type so both bool (true) and
        # launch-passed string ('true') overrides are accepted.
        self.declare_parameter('enabled')
        self.declare_parameter('linear', 0.4)
        self.declare_parameter('angular', 0.6)
        self.declare_parameter('forward_time', 5.0)
        self.declare_parameter('turn_time', 2.6)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.phase = 'forward'
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.tick)

    def auto_enabled(self):
        # ros2 launch passes parameters as strings ('true'/'false'),
        # CLI/file can pass real bools; accept both (None = default on).
        v = self.get_parameter('enabled').value
        if v is None:
            return True
        if isinstance(v, bool):
            return v
        if isinstance(v, str):
            return v.strip().lower() in ('true', '1', 'yes')
        return bool(v)

    def tick(self):
        try:
            enabled = self.auto_enabled()
        except Exception:
            enabled = True
        if not enabled:
            return
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        msg = Twist()
        if self.phase == 'forward':
            msg.linear.x = self.get_parameter('linear').value
            if dt > self.get_parameter('forward_time').value:
                self.phase = 'turn'
                self.t0 = now
        else:
            msg.angular.z = self.get_parameter('angular').value
            if dt > self.get_parameter('turn_time').value:
                self.phase = 'forward'
                self.t0 = now
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(AutoLoop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
