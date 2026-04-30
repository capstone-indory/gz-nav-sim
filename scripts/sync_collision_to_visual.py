#!/usr/bin/env python3
"""SDF 모델의 collision geometry를 visual mesh와 동기화.

Office/hospital 모델은 보통 visual은 정밀 mesh, collision은 단순 도형(box/cylinder)
으로 만들어져 카메라/라이다가 보는 형상과 물리 충돌이 어긋남. 이 스크립트는 각 모델의
visual mesh URI를 그대로 collision으로 복사해 둘을 맞춤.

  사용법:  python3 scripts/sync_collision_to_visual.py [경로 또는 디렉토리]
            기본: src/gz_nav_sim/models/{office,hospital}

  주의:
   - mesh visual이 없는 모델 (visual 자체가 box/cylinder)은 skip
   - 이미 collision이 mesh인 경우 skip
   - 변경 전 .sdf.bak 백업 자동 생성
   - 복잡 mesh는 collision 계산이 느려짐. 시뮬 fps 떨어지면 일부 되돌리세요.
"""
from __future__ import annotations
import sys
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def _find_visual_mesh(link_el: ET.Element) -> tuple[str, str] | None:
    """link element에서 첫 visual mesh의 (uri, scale) 추출. 없으면 None."""
    for visual in link_el.findall('visual'):
        geom = visual.find('geometry')
        if geom is None:
            continue
        mesh = geom.find('mesh')
        if mesh is None:
            continue
        uri_el = mesh.find('uri')
        if uri_el is None or not uri_el.text:
            continue
        scale_el = mesh.find('scale')
        scale = scale_el.text if scale_el is not None else None
        return uri_el.text.strip(), (scale.strip() if scale else None)
    return None


def _patch_collision(link_el: ET.Element, mesh_uri: str, mesh_scale: str | None) -> int:
    """link의 모든 collision geometry를 mesh로 교체. 변경 개수 반환."""
    changed = 0
    for col in link_el.findall('collision'):
        geom = col.find('geometry')
        if geom is None:
            continue
        # 이미 mesh면 skip
        existing_mesh = geom.find('mesh')
        if existing_mesh is not None:
            continue
        # 기존 자식 다 비우고 mesh 삽입
        for child in list(geom):
            geom.remove(child)
        mesh_el = ET.SubElement(geom, 'mesh')
        uri_el = ET.SubElement(mesh_el, 'uri')
        uri_el.text = mesh_uri
        if mesh_scale:
            scale_el = ET.SubElement(mesh_el, 'scale')
            scale_el.text = mesh_scale
        changed += 1
    return changed


def process_sdf(path: Path) -> tuple[int, str]:
    """SDF 파일 하나 처리. (변경 collision 수, 사유) 반환."""
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        return 0, f'parse error: {e}'
    root = tree.getroot()
    total_changed = 0
    for link in root.iter('link'):
        vm = _find_visual_mesh(link)
        if vm is None:
            continue
        uri, scale = vm
        total_changed += _patch_collision(link, uri, scale)
    if total_changed > 0:
        bak = path.with_suffix(path.suffix + '.bak')
        if not bak.exists():
            shutil.copy2(path, bak)
        # ElementTree write — 들여쓰기 유지 위해 indent (Python 3.9+)
        ET.indent(tree, space='  ')
        tree.write(path, encoding='utf-8', xml_declaration=True)
        return total_changed, 'ok'
    return 0, 'no mesh visual or already mesh collision'


def main(argv: list[str]) -> int:
    targets: list[Path] = []
    if len(argv) > 1:
        targets = [Path(p) for p in argv[1:]]
    else:
        repo = Path(__file__).resolve().parent.parent
        for sub in ('src/gz_nav_sim/models/office', 'src/gz_nav_sim/models/hospital'):
            d = repo / sub
            if d.exists():
                targets.append(d)

    sdf_files: list[Path] = []
    for t in targets:
        if t.is_file() and t.suffix == '.sdf':
            sdf_files.append(t)
        elif t.is_dir():
            sdf_files.extend(t.rglob('model.sdf'))

    n_models = len(sdf_files)
    n_patched = 0
    n_collision_changed = 0
    skipped_reasons: dict[str, int] = {}

    for sdf in sorted(sdf_files):
        changed, reason = process_sdf(sdf)
        if changed > 0:
            n_patched += 1
            n_collision_changed += changed
            print(f'[ok] {sdf.relative_to(Path.cwd())} : {changed} collision(s) → mesh')
        else:
            skipped_reasons[reason] = skipped_reasons.get(reason, 0) + 1

    print()
    print(f'총 SDF: {n_models}')
    print(f'패치된 모델: {n_patched}  (collision 교체 {n_collision_changed}개)')
    for reason, cnt in skipped_reasons.items():
        print(f'  skip ({cnt}): {reason}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
