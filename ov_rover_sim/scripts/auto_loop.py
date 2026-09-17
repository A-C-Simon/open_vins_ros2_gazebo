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
        self.declare_parameter('mode', 'circle')
        self.declare_parameter('linear', 0.35)
        self.declare_parameter('angular', 0.6)
        self.declare_parameter('circle_angular', 0.3)
        self.declare_parameter('start_delay', 15.0)
        self.declare_parameter('forward_time', 5.0)
        self.declare_parameter('turn_time', 2.6)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.phase = 'forward'
        self.t0 = self.get_clock().now()
        # Immutable birth stamp for the start delay. NEVER reset this one:
        # the delay branch below holds t0 (the phase clock), so measuring
        # the delay against t0 freezes the measurement and the rover would
        # sit still forever (exactly the bug we just had).
        self.t_start = self.t0
        self.timer = self.create_timer(0.05, self.tick)
        # Slew-limited command shaping: step changes excite chassis
        # oscillation, so ramp toward phase targets instead of jumping.
        self.v = 0.0
        self.w = 0.0

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
        # Circle mode (default): constant gentle turn while translating.
        # This is what VIO needs: the dynamic initializer requires some
        # rotation to succeed, but stop-turns in place give rotation with
        # zero translation (no parallax), so init kept locking onto turn
        # windows with garbage velocity. A circle gives both at all times.
        # Radius 1.17 m (0.35 / 0.3). Proven to lap cleanly with margin:
        # even ballooned ~30 percent wide it stays clear of every pillar.
        # Wider circles reach pillar3. Translation-vs-rotation flow (R/Z)
        # is handled by keeping pillars CLOSE to this path instead.
        # Start delay: hold zero velocity for the first seconds so the
        # freshly spawned model drops, settles and makes clean contact
        # BEFORE any wheel is driven. Driving through the spawn drop
        # kicks/flips the rover (single-wheel contact at speed), after
        # which odometry is fantasy and VIO is hopeless. t0 is held so
        # the square phases start only after the delay.
        now = self.get_clock().now()
        dt = (now - self.t0).nanoseconds / 1e9
        dt_delay = (now - self.t_start).nanoseconds / 1e9
        if dt_delay < self.get_parameter('start_delay').value:
            self.t0 = now
            tv, tw = 0.0, 0.0
        elif self.get_parameter('mode').value == 'circle':
            tv, tw = (self.get_parameter('linear').value,
                      self.get_parameter('circle_angular').value)
        elif self.phase == 'forward':
            tv, tw = self.get_parameter('linear').value, 0.0
            if dt > self.get_parameter('forward_time').value:
                self.phase = 'turn'
                self.t0 = now
        else:
            tv, tw = 0.0, self.get_parameter('angular').value
            if dt > self.get_parameter('turn_time').value:
                self.phase = 'forward'
                self.t0 = now
        step = 0.05  # timer period: slew toward targets, never step
        # Gentle rates: the wheels are very light next to the
        # body, so any sharp torque demand spins the wheels up
        # explosively (28 rad/s seen) faster than the body can follow,
        # and the rebound wheelies/flips the rover. Soft driver on top
        # of soft torque limits: nothing may demand faster than the
        # contact patch can answer.
        for attr, tgt, rate in (('v', tv, 0.3), ('w', tw, 0.5)):
            cur = getattr(self, attr)
            dv = tgt - cur
            lim = rate * step
            setattr(self, attr, cur + max(-lim, min(lim, dv)))
        msg = Twist()
        msg.linear.x, msg.angular.z = self.v, self.w
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(AutoLoop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
