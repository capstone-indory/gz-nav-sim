#!/usr/bin/env python3
"""Simple P-controller drive to (x, y) with timeout.

사용법:
  python3 bench/drive_to.py --x 30 --y 8 --timeout 60 [--vmax 0.6 --wmax 0.8]

도착 (dist < tol) 또는 timeout 시 cmd_vel=0 publish 후 종료.
exit code: 0=success, 1=timeout, 2=error.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class DriveTo(Node):
    def __init__(self, gx, gy, vmax, wmax, tol):
        super().__init__('drive_to')
        self.gx, self.gy = gx, gy
        self.vmax, self.wmax = vmax, wmax
        self.tol = tol
        self.cur = None  # (x, y, yaw)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz
        self.reached = False
        self.start_t = time.time()

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.cur = (p.x, p.y, yaw)

    def _tick(self):
        if self.cur is None:
            return
        x, y, yaw = self.cur
        dx = self.gx - x
        dy = self.gy - y
        dist = math.hypot(dx, dy)
        if dist < self.tol:
            if not self.reached:
                self.get_logger().info(
                    f'reached: cur=({x:.2f},{y:.2f}) goal=({self.gx},{self.gy}) '
                    f'dist={dist:.2f}m elapsed={time.time()-self.start_t:.1f}s')
                self.reached = True
            self.pub.publish(Twist())  # stop
            rclpy.shutdown()
            return
        # P controller — heading first, then drive
        target_yaw = math.atan2(dy, dx)
        yaw_err = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        wz = max(-self.wmax, min(self.wmax, 1.5 * yaw_err))
        # 직진 — yaw error 클 땐 속도 줄임
        vx = self.vmax * max(0.0, math.cos(yaw_err))
        vx = min(vx, dist * 0.5)  # 가까워지면 천천히
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)
        if int(time.time() - self.start_t) % 5 == 0 and time.time() - self.start_t > 1:
            self.get_logger().info(
                f'cur=({x:.2f},{y:.2f}) goal=({self.gx},{self.gy}) '
                f'dist={dist:.2f}m yaw_err={math.degrees(yaw_err):+.1f}° '
                f'cmd vx={vx:.2f} wz={wz:+.2f}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--x', type=float, required=True)
    p.add_argument('--y', type=float, required=True)
    p.add_argument('--timeout', type=float, default=60.0)
    p.add_argument('--vmax', type=float, default=0.6)
    p.add_argument('--wmax', type=float, default=0.8)
    p.add_argument('--tol', type=float, default=0.5)
    args = p.parse_args()

    rclpy.init()
    node = DriveTo(args.x, args.y, args.vmax, args.wmax, args.tol)

    # Publisher가 subscriber를 발견할 때까지 대기 (DDS discovery 지연)
    for _ in range(50):
        if node.pub.get_subscription_count() > 0:
            node.get_logger().info(
                f'publisher matched {node.pub.get_subscription_count()} subscriber(s)')
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    else:
        node.get_logger().warn('no /cmd_vel subscriber found in 5s — sending anyway')

    deadline = time.time() + args.timeout

    try:
        while rclpy.ok() and time.time() < deadline and not node.reached:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # stop robot
        node.pub.publish(Twist())
        time.sleep(0.2)
        node.pub.publish(Twist())
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    if node.reached:
        print(f'[drive_to] SUCCESS reached ({args.x},{args.y})', file=sys.stderr)
        sys.exit(0)
    else:
        print(f'[drive_to] TIMEOUT after {args.timeout}s', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
