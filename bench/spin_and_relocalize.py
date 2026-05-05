#!/usr/bin/env python3
"""한 바퀴 회전 + RTAB-Map BoW 매칭 글로벌 재로컬라이제이션.

가정: rtabmap 이 이미 localization 모드 (Mem/IncrementalMemory=false) 로 진입했다.
  ros_adapter 의 /api/robots/.../slam/relocalize 라우트가 이 스크립트 호출 직전에
  /rtabmap/set_mode_localization 을 호출함.

동작:
  1) /cmd_vel 에 angular.z=0.6 rad/s publish (~10초간 ≈ 2π+α 회전)
  2) /rtabmap/info 의 loop_closure_id 또는 proximity_detection_id 가 0 이상이 되면 수렴.
  3) 수렴 시: cmd_vel=0, exit 0. timeout 시: cmd_vel=0, exit 1.

사용:
  python3 bench/spin_and_relocalize.py [--timeout 15] [--wz 0.6]
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
except ImportError:
    RtabmapInfo = None


class SpinReloc(Node):
    def __init__(self, wz: float, timeout: float):
        super().__init__('spin_and_relocalize')
        self.wz = wz
        self.timeout = timeout
        self.start_t = time.time()
        self.converged = False
        self.last_loop_id = 0
        self.last_prox_id = 0

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        if RtabmapInfo is not None:
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(
                RtabmapInfo, '/rtabmap/info', self._info_cb, qos)
        else:
            self.get_logger().warn(
                'rtabmap_msgs not importable — convergence detection disabled, will spin full timeout')

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

    def _info_cb(self, msg) -> None:
        loop_id = getattr(msg, 'loop_closure_id', 0) or 0
        prox_id = getattr(msg, 'proximity_detection_id', 0) or 0
        if (loop_id > 0 and loop_id != self.last_loop_id) \
                or (prox_id > 0 and prox_id != self.last_prox_id):
            self.last_loop_id = loop_id
            self.last_prox_id = prox_id
            self.get_logger().info(
                f'CONVERGED: loop_id={loop_id} prox_id={prox_id}')
            self.converged = True

    def _tick(self) -> None:
        elapsed = time.time() - self.start_t
        if self.converged or elapsed > self.timeout:
            self._stop()
            return
        cmd = Twist()
        cmd.angular.z = self.wz
        self.pub.publish(cmd)

    def _stop(self) -> None:
        cmd = Twist()
        self.pub.publish(cmd)
        elapsed = time.time() - self.start_t
        if self.converged:
            self.get_logger().info(f'success after {elapsed:.1f}s')
            rclpy.shutdown()
            sys.exit(0)
        else:
            self.get_logger().warn(f'TIMEOUT after {elapsed:.1f}s without convergence')
            rclpy.shutdown()
            sys.exit(1)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--timeout', type=float, default=15.0)
    p.add_argument('--wz', type=float, default=0.6, help='angular.z rad/s')
    args = p.parse_args()

    rclpy.init()
    node = SpinReloc(wz=args.wz, timeout=args.timeout)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop()
    except SystemExit:
        raise


if __name__ == '__main__':
    main()
