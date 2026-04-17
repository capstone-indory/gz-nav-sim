#!/usr/bin/env python3

"""Elevator teleport for the combined office↔hospital world.

Publish std_msgs/Empty on `/elevator/call` while the robot stands inside the
office elevator zone or a hospital elevator cabin. The node teleports the
robot to the other building via the Gazebo `/world/<name>/set_pose` service,
then cycles slam_toolbox through deactivate → cleanup → configure → activate
so SLAM starts a fresh map, and clears the Nav2 costmaps.

Building loading/unloading is handled by the gz-sim Level System
(--levels flag). This node only moves the robot; the level manager
detects the new position and loads the destination building automatically.

Zones and teleport landings live in BUILDINGS below. The office side is a
visual-only elevator in the ServiceSim `walls` model (no real cabin), so we
put both the trigger box and the landing pose around the door texture at
local origin. The hospital side uses AWS RoboMaker elevator_01_car_1, offset
by HOSPITAL_OFFSET_X from the merged world.
"""

from __future__ import annotations

import math
import subprocess
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from std_msgs.msg import Empty


@dataclass
class Zone:
    name: str
    box: tuple[float, float, float, float]  # x_min, x_max, y_min, y_max
    landing: tuple[float, float, float]     # x, y, yaw


# Office elevator is a painted wall at world origin (ServiceSim/Elevator
# material on the `walls` model, local ±0.75 on X at Y=0). Landing drops the
# robot 1m in front of the door facing the office interior.
# Hospital elevator cabin 1 center after merge: (148.5, 19.35).
BUILDINGS = {
    'office':   Zone(name='office',
                     box=(-1.5, 1.5, -1.5, 1.5),
                     landing=(-3.0, 0.0, 0.0)),          # inside office, face +X toward elevator
    'hospital': Zone(name='hospital',
                     box=(146.5, 150.5, 18.0, 20.5),
                     landing=(148.5, 19.3, -1.5708)),    # face -Y out of cabin
}


class ElevatorTeleport(Node):
    def __init__(self) -> None:
        super().__init__('elevator_teleport')
        self.declare_parameter('world', 'combined')
        self.declare_parameter('robot_model', 'robot')
        self.declare_parameter('cooldown_s', 6.0)
        self.declare_parameter('spawn_z', 0.12)
        self.declare_parameter('slam_node', '/slam_toolbox')
        self.declare_parameter('level_settle_s', 2.0)
        self.world = self.get_parameter('world').value
        self.robot_model = self.get_parameter('robot_model').value
        self.cooldown = float(self.get_parameter('cooldown_s').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.slam_node = str(self.get_parameter('slam_node').value).rstrip('/')
        self._level_settle = float(self.get_parameter('level_settle_s').value)

        self._pose_lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._busy_until = 0.0

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Empty, '/elevator/call', self._call_cb, 10)

        self._cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._gmap_clear = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self._lmap_clear = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self._slam_change = self.create_client(
            ChangeState, f'{self.slam_node}/change_state')

        self.get_logger().info(
            "elevator_teleport ready (level system handles building swap). "
            "Publish std_msgs/Empty on /elevator/call from inside an elevator zone.")

    # ── odom ──────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._pose_lock:
            self._x = msg.pose.pose.position.x
            self._y = msg.pose.pose.position.y
            self._yaw = yaw

    def _current_building(self) -> str | None:
        with self._pose_lock:
            x, y = self._x, self._y
        for key, zone in BUILDINGS.items():
            x0, x1, y0, y1 = zone.box
            if x0 <= x <= x1 and y0 <= y <= y1:
                return key
        return None

    # ── call handling ─────────────────────────────────────────────────
    def _call_cb(self, _msg: Empty) -> None:
        now = time.time()
        if now < self._busy_until:
            self.get_logger().warn("call ignored: cooldown active")
            return

        src = self._current_building()
        if src is None:
            with self._pose_lock:
                x, y = self._x, self._y
            self.get_logger().warn(
                f"call ignored: robot at ({x:.2f}, {y:.2f}) not inside any elevator zone")
            return

        dst = 'hospital' if src == 'office' else 'office'
        tx, ty, tyaw = BUILDINGS[dst].landing
        self._busy_until = now + self.cooldown
        self.get_logger().info(f"teleport {src} → {dst}  target=({tx:.2f}, {ty:.2f})")

        self._cmd_vel.publish(Twist())  # stop

        if not self._gz_set_pose(tx, ty, tyaw):
            self.get_logger().error("teleport aborted: gz set_pose failed")
            self._busy_until = 0.0
            return

        # Wait for the level system to load the destination building.
        time.sleep(self._level_settle)
        self._restart_slam()
        self._clear_costmaps()
        self.get_logger().info("teleport done")

    # ── gz service call ───────────────────────────────────────────────
    def _gz_set_pose(self, x: float, y: float, yaw: float) -> bool:
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = (
            f'name: "{self.robot_model}", '
            f'position: {{x: {x}, y: {y}, z: {self.spawn_z}}}, '
            f'orientation: {{x: 0, y: 0, z: {qz}, w: {qw}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req,
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"gz service exception: {exc}")
            return False
        if result.returncode != 0:
            self.get_logger().error(
                f"gz service rc={result.returncode} stderr={result.stderr.strip()}")
            return False
        return True

    # ── slam lifecycle restart ────────────────────────────────────────
    def _slam_transition(self, transition_id: int, label: str) -> bool:
        if not self._slam_change.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(f"slam change_state not ready for {label}")
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self._slam_change.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().warn(f"slam {label} timed out")
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().warn(f"slam {label} failed")
            return False
        return True

    def _restart_slam(self) -> None:
        # active → inactive → unconfigured → inactive → active
        seq = (
            (Transition.TRANSITION_DEACTIVATE, 'deactivate'),
            (Transition.TRANSITION_CLEANUP,    'cleanup'),
            (Transition.TRANSITION_CONFIGURE,  'configure'),
            (Transition.TRANSITION_ACTIVATE,   'activate'),
        )
        for tid, label in seq:
            if not self._slam_transition(tid, label):
                self.get_logger().warn(f"slam restart stopped at {label}")
                return
            time.sleep(0.1)
        self.get_logger().info("slam restarted")

    def _clear_costmaps(self) -> None:
        req = ClearEntireCostmap.Request()
        for client, name in ((self._gmap_clear, 'global'),
                             (self._lmap_clear, 'local')):
            if client.service_is_ready():
                client.call_async(req)
            else:
                self.get_logger().warn(f"{name} costmap clear service not ready")


def main() -> None:
    rclpy.init()
    node = ElevatorTeleport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
