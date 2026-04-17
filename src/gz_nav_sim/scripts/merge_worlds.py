#!/usr/bin/env python3

"""Merge office.world + hospital.world into combined.world (Gazebo Classic 11).

Office is rotated around Z by OFFICE_ROTATE_Z (default +π/2 CCW) so its long
axis swings into −Y, keeping the painted elevator wall at the world origin.
Hospital models are offset by HOSPITAL_OFFSET_X on +X and prefixed with
`hospital_` to avoid name collisions with office.

This is the Gazebo Classic 11 variant: office.world and hospital.world are
native Gazebo Classic SDFs, so we preserve their physics, materials, and
lights as-is.  No gz-sim system plugins, no OGRE PBR conversion, no level
system, no injected overhead camera — just the geometric merge.

  python3 scripts/merge_worlds.py
"""

from __future__ import annotations

import json
import math
import pathlib
import xml.etree.ElementTree as ET

HOSPITAL_OFFSET_X = 150.0
OFFICE_ROTATE_Z = math.pi / 2.0   # CCW 90° around world Z
TOP_LEVEL_COPY = {'model', 'include', 'light', 'actor'}
MANIFEST_ENTRY_TAGS = {'model', 'include', 'light'}

ROOT = pathlib.Path(__file__).resolve().parents[1]
OFFICE = ROOT / 'worlds' / 'office.world'
HOSPITAL = ROOT / 'worlds' / 'hospital.world'
OUT = ROOT / 'worlds' / 'combined.world'
MANIFEST_OUT = ROOT / 'config' / 'building_manifest.json'


def _pose_vals(text: str | None) -> list[float]:
    vals = [float(p) for p in (text or '').split()]
    while len(vals) < 6:
        vals.append(0.0)
    return vals[:6]


def _format_pose(vals: list[float]) -> str:
    return ' '.join(f'{v:.6g}' for v in vals)


def _offset_toplevel(elem: ET.Element, dx: float) -> None:
    pose = elem.find('pose')
    if pose is None:
        pose = ET.SubElement(elem, 'pose')
        pose.text = _format_pose([dx, 0.0, 0.0, 0.0, 0.0, 0.0])
        return
    vals = _pose_vals(pose.text)
    vals[0] += dx
    pose.text = _format_pose(vals)


def _rotate_z_toplevel(elem: ET.Element, theta: float) -> None:
    if abs(theta) < 1e-9:
        return
    pose = elem.find('pose')
    if pose is None:
        pose = ET.SubElement(elem, 'pose')
        pose.text = _format_pose([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    vals = _pose_vals(pose.text)
    c, s = math.cos(theta), math.sin(theta)
    x, y = vals[0], vals[1]
    vals[0] = c * x - s * y
    vals[1] = s * x + c * y
    vals[5] = vals[5] + theta
    pose.text = _format_pose(vals)


def _parse(path: pathlib.Path) -> ET.ElementTree:
    text = path.read_text(encoding='utf-8').lstrip()
    return ET.ElementTree(ET.fromstring(text))


def _entry_name(elem: ET.Element) -> str | None:
    name = elem.get('name')
    if name:
        return name
    sub = elem.find('name')
    if sub is not None and sub.text:
        return sub.text.strip()
    if elem.tag == 'include':
        uri = elem.find('uri')
        if uri is not None and uri.text:
            return uri.text.rsplit('/', 1)[-1].strip()
    return None


def _pose_of(elem: ET.Element) -> list[float]:
    pose = elem.find('pose')
    return _pose_vals(pose.text if pose is not None else None)


def _is_robot_include(elem: ET.Element) -> bool:
    if elem.tag != 'include':
        return False
    uri = elem.find('uri')
    return uri is not None and (uri.text or '').strip().endswith('robot')


def merge() -> None:
    office_tree = _parse(OFFICE)
    hospital_tree = _parse(HOSPITAL)

    office_world = office_tree.getroot().find('world')
    hospital_world = hospital_tree.getroot().find('world')
    if office_world is None or hospital_world is None:
        raise SystemExit('missing <world> element')

    office_world.set('name', 'combined')

    # gazebo_ros_state plugin exposes /set_entity_state, /get_entity_state
    # services used by elevator_teleport.py.  libgazebo_ros_init is loaded
    # automatically by gzserver.launch.py, but state is not.
    state_plugin = ET.Element(
        'plugin',
        {'name': 'gazebo_ros_state', 'filename': 'libgazebo_ros_state.so'})
    ros_el = ET.SubElement(state_plugin, 'ros')
    ET.SubElement(ros_el, 'namespace').text = '/gazebo'
    ET.SubElement(state_plugin, 'update_rate').text = '10.0'
    office_world.insert(0, state_plugin)

    office_entries: list[dict] = []
    hospital_entries: list[dict] = []

    # Rotate every top-level office entity around world Z.  The walls model
    # with the elevator sits at origin, so it's unchanged; every other model,
    # include, and light swings with it.
    for child in list(office_world):
        if child.tag not in TOP_LEVEL_COPY:
            continue
        _rotate_z_toplevel(child, OFFICE_ROTATE_Z)
        if child.tag not in MANIFEST_ENTRY_TAGS:
            continue
        if _is_robot_include(child):
            continue
        name = _entry_name(child)
        if not name:
            continue
        office_entries.append({'name': name, 'pose': _pose_of(child)})

    for child in list(hospital_world):
        if child.tag not in TOP_LEVEL_COPY:
            continue
        name = child.get('name')
        if name:
            child.set('name', f'hospital_{name}')
        _offset_toplevel(child, HOSPITAL_OFFSET_X)
        office_world.append(child)
        if child.tag not in MANIFEST_ENTRY_TAGS:
            continue
        final_name = _entry_name(child)
        if not final_name:
            continue
        hospital_entries.append({'name': final_name, 'pose': _pose_of(child)})

    ET.indent(office_tree, space='  ')
    office_tree.write(OUT, encoding='utf-8', xml_declaration=True)
    print(f'wrote {OUT} ({OUT.stat().st_size // 1024} KB)')

    MANIFEST_OUT.parent.mkdir(parents=True, exist_ok=True)
    manifest = {'office': office_entries, 'hospital': hospital_entries}
    MANIFEST_OUT.write_text(json.dumps(manifest, indent=2))
    print(f'wrote {MANIFEST_OUT} '
          f'(office={len(office_entries)}, hospital={len(hospital_entries)})')


if __name__ == '__main__':
    merge()
