#!/usr/bin/env python3
"""Bench run 결과 비교 표.

사용법:
  python3 bench/compare.py [run-dir...]
  python3 bench/compare.py                          # 최근 10개 자동 선택
  python3 bench/compare.py runs/20260419_140000_*   # 특정 run들
"""

import json
import sys
from pathlib import Path


BENCH_DIR = Path(__file__).parent
RUNS_DIR = BENCH_DIR / 'runs'


def load_metrics(run_dir: Path) -> dict | None:
    f = run_dir / 'metrics.json'
    if not f.exists():
        return None
    try:
        return json.loads(f.read_text())
    except Exception:
        return None


def fmt(v, kind='num'):
    if v is None:
        return '-'
    if kind == 'num':
        if isinstance(v, float):
            return f'{v:.3f}'
        return str(v)
    if kind == 'pct':
        return f'{v:.0f}%'
    return str(v)


def row(run_dir: Path, m: dict) -> dict:
    da3 = m.get('da3', {})
    vggt = m.get('vggt', {})
    nv = m.get('nvblox', {})
    err = m.get('errors', {})

    s_stat = da3.get('s_stats') or {}
    inf_stat = (da3.get('timing_stats') or {}).get('infer_stats') or {}
    smooth_ds = da3.get('smoothed_ds_stats') or {}

    return {
        'run': run_dir.name,
        'model': da3.get('model_id') or '-',
        'da3_batches': da3.get('affine_count', 0),
        'da3_s_min': s_stat.get('min'),
        'da3_s_max': s_stat.get('max'),
        'da3_s_jitter': (s_stat.get('max', 0) - s_stat.get('min', 0)) if s_stat else None,
        'da3_smooth_ds_max': smooth_ds.get('max'),
        'da3_infer_mean_s': inf_stat.get('mean'),
        'vggt_submaps': vggt.get('submap_count', 0),
        'vggt_lock_scale': vggt.get('global_lock_scale'),
        'nvblox_depth_n': nv.get('depth_integrated_total', 0),
        'nvblox_mesh_n': nv.get('mesh_integrated_total', 0),
        'errors': err.get('count', 0),
    }


def main():
    args = sys.argv[1:]
    if not args:
        runs = sorted(RUNS_DIR.glob('*/'), key=lambda p: p.name)[-10:]
    else:
        runs = []
        for a in args:
            p = Path(a)
            if not p.is_absolute():
                p = BENCH_DIR / p
            if p.is_dir():
                runs.append(p)
            elif '*' in str(p):
                runs.extend(sorted(Path('.').glob(str(p))))

    if not runs:
        print('no runs found')
        sys.exit(1)

    rows = []
    for r in runs:
        m = load_metrics(r)
        if m is None:
            continue
        rows.append(row(r, m))

    if not rows:
        print('no metrics found in any run')
        sys.exit(1)

    cols = [
        ('run', 30), ('model', 25),
        ('da3_batches', 8),
        ('da3_s_min', 9), ('da3_s_max', 9), ('da3_s_jitter', 10),
        ('da3_smooth_ds_max', 12), ('da3_infer_mean_s', 12),
        ('vggt_submaps', 8), ('vggt_lock_scale', 12),
        ('nvblox_depth_n', 10), ('nvblox_mesh_n', 10),
        ('errors', 6),
    ]
    header = ' | '.join(f'{c:<{w}}' for c, w in cols)
    sep = '-' * len(header)
    print(sep)
    print(header)
    print(sep)
    for r in rows:
        print(' | '.join(f'{fmt(r.get(c)):<{w}}' for c, w in cols))
    print(sep)


if __name__ == '__main__':
    main()
