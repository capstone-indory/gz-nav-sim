#!/usr/bin/env python3
"""nvblox mesh → Foxglove SceneUpdate (TriangleListPrimitive) republisher.

nvblox_msgs/Mesh는 변경된 블록만 incremental delta로 옴. 이를 블록별
SceneEntity + TriangleListPrimitive로 변환해 송신.

TriangleListPrimitive는 Foxglove가 native로 렌더링 (인덱싱 지원). 따라서
glTF/Draco wrapper 없이도 mesh_marker(중복 정점)의 1/6 수준 대역폭.

입력:  /nvblox_node/mesh    nvblox_msgs/Mesh
출력:  /nvblox_node/scene   foxglove_msgs/SceneUpdate
"""

from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from foxglove_msgs.msg import (Color, SceneEntity, SceneEntityDeletion,
                               SceneUpdate, TriangleListPrimitive)
from geometry_msgs.msg import Point, Pose
from nvblox_msgs.msg import Mesh
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MeshToSceneRepublisher(Node):
    def __init__(self) -> None:
        super().__init__('nvblox_mesh_to_gltf')

        self.declare_parameter('input_topic', '/nvblox_node/mesh')
        self.declare_parameter('output_topic', '/nvblox_node/scene')
        self.declare_parameter('frame_id_override', '')
        # true: nvblox가 메모리 절약차 블록을 잘라도 Foxglove에선 누적 유지
        # false: nvblox 삭제를 Foxglove에도 그대로 반영
        # accumulate_only=true 의 부작용: nvblox 가 block 을 메모리 정리로 잘라도
        # SceneUpdate 에 deletion 안 발행 → 다운스트림 cache (foxglove / adapter)
        # 가 _영구 누적_ → 새 client 가 connect 할 때마다 옛 잔재까지 모두 받음 →
        # 매 새로고침 시 시각 더 더러워지는 현상. default 를 false 로 — nvblox 의
        # block 정리 신호를 그대로 따라가 cache 가 자연 정리됨.
        self.declare_parameter('accumulate_only', False)
        # 단일 SceneUpdate frame 의 대략적 byte 한도. 17~23MB 짜리 single frame은
        # WebSocket client 들의 default frame size limit (1~10MB) 에 걸려 즉시 끊김.
        # 한 nvblox callback 에 들어온 블록들을 이 한도 단위로 분할 publish 해야
        # web 에서 안정적으로 수신 가능. 1MB 면 거의 모든 client 가 받음.
        self.declare_parameter('max_frame_bytes', 1_000_000)

        in_topic = str(self.get_parameter('input_topic').value)
        out_topic = str(self.get_parameter('output_topic').value)
        self._frame_override = str(self.get_parameter('frame_id_override').value).strip()
        self._accumulate_only = bool(self.get_parameter('accumulate_only').value)
        self._max_frame_bytes = int(self.get_parameter('max_frame_bytes').value)

        sub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE)
        # TRANSIENT_LOCAL: late subscriber도 latest 메시지 즉시 받음 (latched).
        # VOLATILE이면 Foxglove client가 연결 후 다음 publish (0.2Hz=5초+)까지 빈 화면.
        # mesh는 누적이라 latest 1개로 전체 scene 재구성 가능 → depth=1 충분.
        pub_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._sub = self.create_subscription(Mesh, in_topic, self._on_mesh, sub_qos)
        self._pub = self.create_publisher(SceneUpdate, out_topic, pub_qos)

        self._known_block_ids: set[str] = set()
        self._stats = {'msgs': 0, 'in_bytes': 0, 'out_bytes': 0,
                       'blocks_encoded': 0, 'blocks_deleted': 0}

        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f'mesh_to_scene: {in_topic} → {out_topic} '
            f'(TriangleListPrimitive, indexed)'
        )

    def _on_mesh(self, msg: Mesh) -> None:
        self._stats['msgs'] += 1
        self._stats['in_bytes'] += sum(
            (len(b.vertices) * 12 + len(b.colors) * 16 + len(b.triangles) * 4)
            for b in msg.blocks
        )

        update = SceneUpdate()
        frame_id = self._frame_override or msg.header.frame_id
        approx_out = 0
        # 현재 진행 중인 batch 의 누적 byte. max_frame_bytes 초과 시 publish + reset.
        batch_bytes = 0

        if msg.clear:
            # 전체 리셋 신호. accumulate_only면 무시하고 누적 유지.
            if not self._accumulate_only:
                for old_id in list(self._known_block_ids):
                    deletion = SceneEntityDeletion()
                    deletion.timestamp = msg.header.stamp
                    deletion.type = SceneEntityDeletion.MATCHING_ID
                    deletion.id = old_id
                    update.deletions.append(deletion)
                self._known_block_ids.clear()

        for idx, block in zip(msg.block_indices, msg.blocks):
            entity_id = f'b_{idx.x}_{idx.y}_{idx.z}'
            n_verts = len(block.vertices)
            n_tris = len(block.triangles) // 3

            if n_tris == 0 or n_verts == 0:
                # nvblox가 블록을 잘라냈음. accumulate_only면 Foxglove에선 유지.
                if not self._accumulate_only and entity_id in self._known_block_ids:
                    deletion = SceneEntityDeletion()
                    deletion.timestamp = msg.header.stamp
                    deletion.type = SceneEntityDeletion.MATCHING_ID
                    deletion.id = entity_id
                    update.deletions.append(deletion)
                    self._known_block_ids.discard(entity_id)
                    self._stats['blocks_deleted'] += 1
                continue

            entity = SceneEntity()
            entity.timestamp = msg.header.stamp
            entity.frame_id = frame_id
            entity.id = entity_id
            entity.lifetime = DurationMsg()  # 0 = 영구
            entity.frame_locked = False

            tri = TriangleListPrimitive()
            tri.pose = Pose()
            tri.pose.orientation.w = 1.0

            # 정점: nvblox vertices는 world(global_frame) 좌표
            tri.points = [Point(x=float(v.x), y=float(v.y), z=float(v.z))
                          for v in block.vertices]

            # 색상: ColorRGBA(0..1) — 그대로 foxglove Color로 전달
            if len(block.colors) == n_verts:
                tri.colors = [Color(r=float(c.r), g=float(c.g),
                                    b=float(c.b), a=float(c.a))
                              for c in block.colors]
            else:
                tri.color = Color(r=0.7, g=0.7, b=0.7, a=1.0)

            # 인덱스: nvblox triangles는 (3M,) int32, 그대로 평탄 list로
            tri.indices = [int(i) for i in block.triangles]

            entity.triangles.append(tri)
            update.entities.append(entity)

            self._known_block_ids.add(entity_id)
            self._stats['blocks_encoded'] += 1
            # 대략적 출력 크기 (Point=24B, Color=16B, idx=4B per vertex/index)
            entity_bytes = n_verts * (24 + 16) + len(block.triangles) * 4
            approx_out += entity_bytes
            batch_bytes += entity_bytes

            # batch 가 한도 초과하면 즉시 flush — 다음 entity 는 새 SceneUpdate 로.
            # 이래야 single WebSocket frame 이 max_frame_bytes 이하로 유지돼
            # web client default limit 에 걸리지 않음.
            if batch_bytes >= self._max_frame_bytes:
                self._pub.publish(update)
                update = SceneUpdate()
                batch_bytes = 0

        self._stats['out_bytes'] += approx_out

        # 마지막 잔여 batch + (entities 없이 deletions 만 있는 케이스 포함) flush.
        if update.entities or update.deletions:
            self._pub.publish(update)

    def _log_stats(self) -> None:
        s = self._stats
        if s['msgs'] == 0:
            return
        ratio = (s['in_bytes'] / s['out_bytes']) if s['out_bytes'] > 0 else 0.0
        self.get_logger().info(
            f"msgs={s['msgs']}  blocks(enc={s['blocks_encoded']} "
            f"del={s['blocks_deleted']})  "
            f"in={s['in_bytes']/1e6:.1f}MB out={s['out_bytes']/1e6:.1f}MB "
            f"ratio={ratio:.2f}x  active_blocks={len(self._known_block_ids)}"
        )


def main() -> None:
    rclpy.init()
    node = MeshToSceneRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
