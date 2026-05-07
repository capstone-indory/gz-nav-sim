#!/usr/bin/env python3
"""Subscribe once to /map (SLAM occupancy grid) and save."""
import os, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

OUT = os.path.dirname(os.path.abspath(__file__))


class Grab(Node):
    def __init__(self):
        super().__init__('map_grabber')
        self.got = None
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/map', self._got, qos)

    def _got(self, msg):
        if self.got is not None:
            return
        h, w = msg.info.height, msg.info.width
        arr = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self.got = (arr, {
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'width': w, 'height': h,
            'frame_id': msg.header.frame_id,
        })
        self.get_logger().info(f'/map: {w}x{h} @ {msg.info.resolution}m')


def main():
    rclpy.init()
    node = Grab()
    deadline = time.time() + 12
    while time.time() < deadline and node.got is None:
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.got is None:
        print('TIMEOUT: no /map message received')
    else:
        arr, meta = node.got
        path = os.path.join(OUT, 'map_slam.npz')
        np.savez(path, grid=arr, **meta)
        print(f'saved {path}: shape={arr.shape}, frame={meta["frame_id"]}')
        unknown = (arr == -1).sum()
        free = (arr == 0).sum()
        occ = (arr > 50).sum()
        print(f'  unknown={unknown}  free={free}  occupied={occ}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
