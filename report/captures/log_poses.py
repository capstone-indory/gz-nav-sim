#!/usr/bin/env python3
"""Log GT (Gazebo) + odometry + SLAM pose simultaneously.

Saves pose triples to captures/poses.npz.
SLAM pose comes from TF (map -> base_link) since /pose is occasional only.
"""
import os, time, math, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException

OUT = os.path.dirname(os.path.abspath(__file__))
ROBOT = os.environ.get('ROBOT_NAME', 'robot_depth_sensor')
DURATION = float(os.environ.get('LOG_SECONDS', '90'))


def quat_to_yaw(qx, qy, qz, qw):
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)


class Logger(Node):
    def __init__(self):
        super().__init__('pose_logger')
        self.gt = None
        self.odom = None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(ModelStates, '/gazebo/model_states', self._gt, qos)
        self.create_subscription(Odometry, '/odom', self._odom, qos)
        self.records = []
        self.t0 = None
        self.create_timer(0.1, self._tick)

    def _gt(self, msg):
        try:
            i = list(msg.name).index(ROBOT)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        self.gt = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def _odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom = (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))

    def _slam(self):
        try:
            t = self.tf_buf.lookup_transform('map', 'base_link',
                                              rclpy.time.Time(),
                                              timeout=rclpy.duration.Duration(seconds=0.05))
            p = t.transform.translation
            q = t.transform.rotation
            return (p.x, p.y, quat_to_yaw(q.x, q.y, q.z, q.w))
        except (LookupException, ExtrapolationException):
            return None

    def _tick(self):
        if self.gt is None or self.odom is None:
            return
        slam = self._slam()
        if slam is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.records.append((now - self.t0, *self.gt, *self.odom, *slam))
        if (now - self.t0) >= DURATION:
            print(f'logged {len(self.records)} samples over {DURATION:.0f}s')
            self.save()
            rclpy.shutdown()

    def save(self):
        arr = np.array(self.records, dtype=np.float64)
        path = os.path.join(OUT, 'poses.npz')
        np.savez(path, data=arr,
                 columns=np.array(['t', 'gt_x', 'gt_y', 'gt_yaw',
                                   'odom_x', 'odom_y', 'odom_yaw',
                                   'slam_x', 'slam_y', 'slam_yaw']))
        print(f'saved {path}: shape={arr.shape}')


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
    finally:
        try: rclpy.shutdown()
        except Exception: pass


if __name__ == '__main__':
    main()
