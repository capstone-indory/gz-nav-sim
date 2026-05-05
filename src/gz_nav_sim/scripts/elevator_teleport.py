#!/usr/bin/env python3

"""Elevator teleport for the combined office↔hospital world (Gazebo Classic 11).

Publish std_msgs/Empty on `/elevator/call` while the robot stands inside the
office elevator zone or a hospital elevator cabin. The node teleports the
robot to the other building via the gazebo_ros `/gazebo/set_entity_state`
service, then performs a SLAM backend transition.

Two SLAM backends supported:
- slam_toolbox (legacy): lifecycle deactivate→cleanup→configure→activate
  (fresh map per building).
- rtabmap (default for use_rtabmap=true): per-floor .db swap via
  /rtabmap/load_database, preserving cross-session map state. Each building
  has its own .db file under FLOOR_DB_DIR/{floor_code}.db (pre-staged by the
  ros_adapter on REST /api/robots/.../floor/set).

Zones and teleport landings live in BUILDINGS below. The office side is a
visual-only elevator in the ServiceSim `walls` model (no real cabin), so we
put both the trigger box and the landing pose around the door texture at
local origin. The hospital side uses AWS RoboMaker elevator_01_car_1, offset
by HOSPITAL_OFFSET_X from the merged world.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Empty as EmptySrv


@dataclass
class Zone:
    name: str
    box: tuple[float, float, float, float]  # x_min, x_max, y_min, y_max
    landing: tuple[float, float, float]     # x, y, yaw
    floor_code: str                          # IndoorMap.code 와 매핑


BUILDINGS = {
    'office':   Zone(name='office',
                     box=(-1.5, 1.5, -1.5, 1.5),
                     landing=(-3.0, 0.0, 0.0),
                     floor_code='office'),
    'hospital': Zone(name='hospital',
                     box=(146.5, 150.5, 18.0, 20.5),
                     landing=(148.5, 19.3, -1.5708),
                     floor_code='hospital'),
}


# 어댑터가 stage 한 floor 별 RTAB-Map DB 위치. ros_adapter/main.py 와 일치해야 함.
FLOOR_DB_DIR = os.environ.get('FLOOR_DB_DIR', '/var/indoory/floor_dbs')


class ElevatorTeleport(Node):
    def __init__(self) -> None:
        super().__init__('elevator_teleport')
        self.declare_parameter('robot_model', 'robot')
        self.declare_parameter('cooldown_s', 6.0)
        self.declare_parameter('spawn_z', 0.12)
        self.declare_parameter('slam_node', '/slam_toolbox')
        self.declare_parameter('set_state_service',
                               '/gazebo/set_entity_state')
        # 'slam_toolbox' (라이프사이클 fresh restart) | 'rtabmap' (DB swap, 멀티세션)
        self.declare_parameter('slam_backend', 'slam_toolbox')
        self.robot_model = self.get_parameter('robot_model').value
        self.cooldown = float(self.get_parameter('cooldown_s').value)
        self.spawn_z = float(self.get_parameter('spawn_z').value)
        self.slam_node = str(self.get_parameter('slam_node').value).rstrip('/')
        self.set_state_service = str(
            self.get_parameter('set_state_service').value)
        self.slam_backend = str(self.get_parameter('slam_backend').value).lower()

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
        self._set_state = self.create_client(
            SetEntityState, self.set_state_service)
        # rtabmap 멀티세션 클라이언트 (slam_backend='rtabmap' 일 때만 사용).
        self._rtabmap_backup = self.create_client(EmptySrv, '/rtabmap/backup')
        self._rtabmap_reset = self.create_client(EmptySrv, '/rtabmap/reset')

        self.get_logger().info(
            "elevator_teleport ready. "
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

        if not self._set_entity_pose(tx, ty, tyaw):
            self.get_logger().error("teleport aborted: set_entity_state failed")
            self._busy_until = 0.0
            return

        if self.slam_backend == 'rtabmap':
            self._switch_rtabmap_floor(BUILDINGS[dst].floor_code)
        else:
            self._restart_slam()
        self._clear_costmaps()
        self.get_logger().info("teleport done")

    # ── gazebo_ros service call ───────────────────────────────────────
    def _set_entity_pose(self, x: float, y: float, yaw: float) -> bool:
        if not self._set_state.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                f"{self.set_state_service} service not available")
            return False
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        req = SetEntityState.Request()
        req.state.name = self.robot_model
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = self.spawn_z
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw
        req.state.reference_frame = 'world'
        future = self._set_state.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            self.get_logger().error("set_entity_state call timed out")
            return False
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error(
                f"set_entity_state failed: "
                f"{getattr(result, 'status_message', 'no result')}")
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

    # ── rtabmap 멀티세션 층 전환 ─────────────────────────────────────────
    def _switch_rtabmap_floor(self, floor_code: str) -> None:
        """현재 graph 백업 후 floor_code 의 .db 로 reload.

        DB 파일은 어댑터가 사전에 FLOOR_DB_DIR/{floor_code}.db 로 stage 했다고 가정.
        rtabmap_msgs/srv/LoadDatabase 는 별도 import 필요해 여기서는 ros2 service call
        subprocess 로 처리 (의존성 줄이기 + 환경 sourced 보장).
        """
        target = os.path.join(FLOOR_DB_DIR, f'{floor_code}.db')
        # 1) 현재 working DB 백업 — 데이터 보존.
        if self._rtabmap_backup.service_is_ready():
            self._rtabmap_backup.call_async(EmptySrv.Request())
        # 2) 새 DB 로드. 어댑터가 stage 안 했으면 그냥 reset 만.
        if not os.path.exists(target):
            self.get_logger().warn(
                f"floor db not staged at {target} — resetting working memory only")
            if self._rtabmap_reset.service_is_ready():
                self._rtabmap_reset.call_async(EmptySrv.Request())
            return
        import subprocess
        cmd = (
            f"ros2 service call /rtabmap/load_database "
            f"rtabmap_msgs/srv/LoadDatabase "
            f"'{{database_path: \"{target}\", clear: true}}'"
        )
        self.get_logger().info(f"loading rtabmap db: {target}")
        try:
            r = subprocess.run(['bash', '-c', cmd], timeout=15,
                               capture_output=True, text=True)
            if r.returncode != 0:
                self.get_logger().warn(f"load_database srv failed: {r.stderr[:200]}")
        except Exception as e:
            self.get_logger().warn(f"load_database srv error: {e}")

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
