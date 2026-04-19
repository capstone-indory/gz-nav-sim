#!/usr/bin/env python3
"""Bench metrics extractor.

stdout.log → metrics.json. 알려진 로그 패턴(정규식) 매칭해 메트릭 추출.

사용법:
  python3 extract_metrics.py <stdout.log> [> metrics.json]
"""

import json
import re
import sys
from pathlib import Path


PATTERNS = {
    # DA3 affine fit: [batch_affine] s=1.0012, t=-0.0141 (1/m) inliers=219/480 (46%) z_l=[1.7,3.1]m
    'da3_affine': re.compile(
        r'\[batch_affine\] s=([\d.]+), t=([+-]?[\d.]+) \(1/m\) '
        r'inliers=(\d+)/(\d+) \((\d+)%\) z_l=\[([\d.]+),([\d.]+)\]m'
    ),
    # DA3 smoothing: [smooth] s_raw=1.18→s=1.05 (Δ0.05) t_raw=+0.02→t=+0.00 (Δ0.01)
    'da3_smooth': re.compile(
        r'\[smooth\] s_raw=([\d.]+)→s=([\d.]+) \(Δ([\d.]+)\) '
        r't_raw=([+-]?[\d.]+)→t=([+-]?[\d.]+) \(Δ([\d.]+)\)'
    ),
    # DA3 timing: [timing] decode=0.05s tf=0.10s infer=2.50s lidar+affine=0.05s publish=0.20s TOTAL=2.90s frames=5 buf_after=0
    'da3_timing': re.compile(
        r'\[timing\] decode=([\d.]+)s tf=([\d.]+)s infer=([\d.]+)s '
        r'lidar\+affine=([\d.]+)s publish=([\d.]+)s TOTAL=([\d.]+)s '
        r'frames=(\d+) buf_after=(\d+)'
    ),
    # VGGT submap: [vggt_slam_server] submap 3 processed (frames=8)
    'vggt_submap': re.compile(
        r'\[vggt_slam_server\] submap (\d+) processed \(frames=(\d+)\)'
    ),
    # VGGT global xform lock: [global_xform] LOCKED at first keyframe: scale=1.0234 t_g=[...]
    'vggt_global_lock': re.compile(
        r'\[global_xform\] LOCKED.*scale=([\d.]+).*\(n=(\d+)\)'
    ),
    # nvblox depth integrate (timing block에서)
    'nvblox_depth_count': re.compile(
        r'ros/depth/integrate\s+(\d+)\s+([\d.]+)'
    ),
    'nvblox_color_count': re.compile(
        r'ros/color/integrate\s+(\d+)\s+([\d.]+)'
    ),
    'nvblox_mesh_count': re.compile(
        r'mesh/gpu/integrate\s+(\d+)\s+([\d.]+)'
    ),
    # 모델 로드: Loading DA3 model depth-anything/DA3-LARGE-1.1 on cuda...
    'da3_model_id': re.compile(r'Loading DA3 model (\S+) on'),
    # VGGT-SLAM 준비: [vggt_slam_server] VGGT ready
    'vggt_ready': re.compile(r'\[vggt_slam_server\] VGGT ready'),
    # 에러
    'error': re.compile(r'(?:ERROR|Traceback|FAIL)', re.IGNORECASE),
}


def extract(log_path: Path) -> dict:
    text = log_path.read_text(errors='ignore')
    lines = text.split('\n')

    da3_affine = []
    da3_smooth = []
    da3_timing = []
    vggt_submap = []
    nvblox_depth_last = (0, 0.0)
    nvblox_color_last = (0, 0.0)
    nvblox_mesh_last = (0, 0.0)
    da3_model_id = None
    vggt_ready_at_line = None
    global_lock_scale = None
    global_lock_n = 0
    error_count = 0
    error_samples = []

    for i, line in enumerate(lines):
        m = PATTERNS['da3_affine'].search(line)
        if m:
            da3_affine.append({
                's': float(m[1]), 't': float(m[2]),
                'inliers': int(m[3]), 'total': int(m[4]),
                'inlier_pct': int(m[5]),
                'z_min': float(m[6]), 'z_max': float(m[7]),
            })
            continue
        m = PATTERNS['da3_smooth'].search(line)
        if m:
            da3_smooth.append({
                's_raw': float(m[1]), 's': float(m[2]), 'ds': float(m[3]),
                't_raw': float(m[4]), 't': float(m[5]), 'dt': float(m[6]),
            })
            continue
        m = PATTERNS['da3_timing'].search(line)
        if m:
            da3_timing.append({
                'decode': float(m[1]), 'tf': float(m[2]),
                'infer': float(m[3]), 'lidar': float(m[4]),
                'publish': float(m[5]), 'total': float(m[6]),
                'frames': int(m[7]), 'buf_after': int(m[8]),
            })
            continue
        m = PATTERNS['vggt_submap'].search(line)
        if m:
            vggt_submap.append({
                'submap_id': int(m[1]), 'frames': int(m[2]),
            })
            continue
        m = PATTERNS['vggt_global_lock'].search(line)
        if m:
            global_lock_scale = float(m[1])
            global_lock_n = int(m[2])
            continue
        m = PATTERNS['nvblox_depth_count'].search(line)
        if m:
            nvblox_depth_last = (int(m[1]), float(m[2]))
            continue
        m = PATTERNS['nvblox_color_count'].search(line)
        if m:
            nvblox_color_last = (int(m[1]), float(m[2]))
            continue
        m = PATTERNS['nvblox_mesh_count'].search(line)
        if m:
            nvblox_mesh_last = (int(m[1]), float(m[2]))
            continue
        m = PATTERNS['da3_model_id'].search(line)
        if m:
            da3_model_id = m[1]
            continue
        if PATTERNS['vggt_ready'].search(line):
            vggt_ready_at_line = i
            continue
        if PATTERNS['error'].search(line):
            error_count += 1
            if len(error_samples) < 5:
                error_samples.append(line.strip()[:200])

    # 통계
    def stat(values, key):
        if not values:
            return None
        nums = [v[key] for v in values]
        return {
            'count': len(nums),
            'min': min(nums), 'max': max(nums),
            'mean': sum(nums) / len(nums),
        }

    return {
        'log_path': str(log_path),
        'log_lines': len(lines),
        'da3': {
            'model_id': da3_model_id,
            'affine_count': len(da3_affine),
            's_stats': stat(da3_affine, 's'),
            't_stats': stat(da3_affine, 't'),
            'inlier_pct_stats': stat(da3_affine, 'inlier_pct'),
            'smoothed_s_stats': stat(da3_smooth, 's'),
            'smoothed_ds_stats': stat(da3_smooth, 'ds'),
            'timing_stats': {
                'infer_stats': stat(da3_timing, 'infer'),
                'total_stats': stat(da3_timing, 'total'),
                'tf_stats': stat(da3_timing, 'tf'),
                'publish_stats': stat(da3_timing, 'publish'),
            },
            'first_5_affine': da3_affine[:5],
            'last_5_affine': da3_affine[-5:],
        },
        'vggt': {
            'submap_count': len(vggt_submap),
            'ready_at_log_line': vggt_ready_at_line,
            'global_lock_scale': global_lock_scale,
            'global_lock_inliers': global_lock_n,
        },
        'nvblox': {
            'depth_integrated_total': nvblox_depth_last[0],
            'color_integrated_total': nvblox_color_last[0],
            'mesh_integrated_total': nvblox_mesh_last[0],
            'depth_total_time_s': nvblox_depth_last[1],
        },
        'errors': {
            'count': error_count,
            'samples': error_samples,
        },
    }


def main():
    if len(sys.argv) < 2:
        print(__doc__, file=sys.stderr)
        sys.exit(1)
    log = Path(sys.argv[1])
    if not log.exists():
        print(json.dumps({'error': f'log not found: {log}'}), file=sys.stderr)
        sys.exit(1)
    result = extract(log)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
