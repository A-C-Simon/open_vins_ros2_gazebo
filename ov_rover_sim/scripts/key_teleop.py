#!/usr/bin/env python3
"""Arrow-key teleop for the rover. Publishes geometry_msgs/Twist on /cmd_vel.

  Up/Down    : forward / backward
  Left/Right : turn left / right
  +/-        : faster / slower
  Space      : emergency stop
  Q          : quit

Exclusive ownership: run this OR auto_loop, never both (gazebo_test.sh
enforces exclusive --auto / --teleop modes). A second /cmd_vel writer,
even an idle keyboard node spamming zeros, fights for the topic.
"""
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """\
Arrow-key teleop (/cmd_vel):
  Up/Down     forward/backward      Left/Right  turn
  + / -       speed up/down         Space       stop
  Q           quit
"""

LIN_STEP = 0.1
ANG_STEP = 0.1
MAX_LIN = 1.0
MAX_ANG = 1.5


def get_key(fd, timeout=0.1):
    # NOTE: must use os.read (single byte), NOT sys.stdin.read(1):
    # TextIOWrapper over-reads into a userspace buffer, hiding the rest
    # of an arrow-key escape sequence from the following select() calls.
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return ''
    try:
        ch = os.read(fd, 1).decode('utf-8', 'ignore')
    except OSError:
        return ''
    if ch == '\x1b':  # escape sequence (arrows)
        # generous windows: ros2-run wrappers/SSH can split bytes in time.
        # A lone ESC is ignored (quit with Q); only complete sequences act.
        r, _, _ = select.select([fd], [], [], 0.25)
        if r:
            try:
                ch2 = os.read(fd, 1).decode('utf-8', 'ignore')
            except OSError:
                return ''
            if ch2 == '[':
                r, _, _ = select.select([fd], [], [], 0.25)
                if r:
                    try:
                        return '\x1b[' + os.read(fd, 1).decode('utf-8', 'ignore')
                    except OSError:
                        pass
        return ''
    return ch


class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lin = 0.0
        self.ang = 0.0
        self.max_lin = 0.4
        self.max_ang = 0.6

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.lin
        msg.angular.z = self.ang
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = KeyTeleop()
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(HELP, flush=True)
    last_key = 0.0
    try:
        tty.setraw(fd)
        while rclpy.ok():
            # Momentary drive: hold a key (terminal auto-repeat) to move,
            # release to stop. Published only on change so auto-drive can
            # resume ~2 s after the last keypress.
            k = get_key(fd)
            now = time.monotonic()
            if k == '\x1b[A':
                node.lin, node.ang = node.max_lin, 0.0
            elif k == '\x1b[B':
                node.lin, node.ang = -node.max_lin, 0.0
            elif k == '\x1b[C':
                node.lin, node.ang = 0.0, -node.max_ang
            elif k == '\x1b[D':
                node.lin, node.ang = 0.0, node.max_ang
            elif k == '+':
                node.max_lin = min(node.max_lin + LIN_STEP, MAX_LIN)
                node.max_ang = min(node.max_ang + ANG_STEP, MAX_ANG)
            elif k == '-':
                node.max_lin = max(node.max_lin - LIN_STEP, LIN_STEP)
                node.max_ang = max(node.max_ang - ANG_STEP, ANG_STEP)
            elif k == ' ':
                node.lin = 0.0
                node.ang = 0.0
            elif k in ('q', 'Q', '\x03'):
                break
            if k:
                node.publish_cmd()
                last_key = now
            elif (node.lin or node.ang) and now - last_key > 0.25:
                node.lin = 0.0
                node.ang = 0.0
                node.publish_cmd()
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()
        print('\nstopped.', flush=True)


if __name__ == '__main__':
    main()
