#!/usr/bin/env python3
"""Subscribe once to /global_costmap/costmap and /local_costmap/costmap, save as .npz."""
import os, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid

OUT = os.path.dirname(os.path.abspath(__file__))


class Grab(Node):
    def __init__(self):
        super().__init__('costmap_grabber')
        self.got = {}
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: self._got('global', m), qos)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda m: self._got('local', m), qos)

    def _got(self, name, msg):
        if name in self.got:
            return
        h, w = msg.info.height, msg.info.width
        arr = np.array(msg.data, dtype=np.int8).reshape(h, w)
        meta = {
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'width': w, 'height': h,
            'frame_id': msg.header.frame_id,
        }
        self.got[name] = (arr, meta)
        self.get_logger().info(f'{name}: {w}x{h} @ {msg.info.resolution}m')


def main():
    rclpy.init()
    node = Grab()
    deadline = time.time() + 15
    while time.time() < deadline and len(node.got) < 2:
        rclpy.spin_once(node, timeout_sec=0.5)
    for name, (arr, meta) in node.got.items():
        path = os.path.join(OUT, f'costmap_{name}.npz')
        np.savez(path, grid=arr, **meta)
        print(f'saved {path}: shape={arr.shape}, frame={meta["frame_id"]}, res={meta["resolution"]}')
    if len(node.got) < 2:
        missing = {'global', 'local'} - set(node.got)
        print(f'WARN: missing {missing}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
