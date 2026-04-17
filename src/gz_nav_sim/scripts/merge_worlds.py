#!/usr/bin/env python3

"""Merge office.world + hospital.world into combined.world.

Office is rotated around Z by OFFICE_ROTATE_Z (default +π/2 CCW) so its long
axis swings into −Y, keeping the painted elevator wall at the world origin.
Hospital models are offset by HOSPITAL_OFFSET_X on +X and prefixed with
`hospital_` to avoid name collisions with office. Run once; commit the
result. Re-run after editing either source world.

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

# gz-sim 8 system plugins. Office/hospital sources are Gazebo Classic and
# carry none of these, so the sensors system never ticks and /scan /camera
# stay empty. Inject explicitly.
GZ_SYSTEM_PLUGINS = [
    ('gz-sim-physics-system',           'gz::sim::systems::Physics'),
    ('gz-sim-user-commands-system',     'gz::sim::systems::UserCommands'),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster'),
    ('gz-sim-sensors-system',           'gz::sim::systems::Sensors'),
]

ROOT = pathlib.Path(__file__).resolve().parents[1]
OFFICE = ROOT / 'worlds' / 'office.world'
HOSPITAL = ROOT / 'worlds' / 'hospital.world'
OUT = ROOT / 'worlds' / 'combined.world'
MANIFEST_OUT = ROOT / 'config' / 'building_manifest.json'
MATERIAL_FILE = (ROOT / 'models' / 'office' / 'media' / 'materials'
                 / 'scripts' / 'servicesim.material')
TEXTURE_URI_BASE = 'file://media/materials/textures'


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
    # Some source SDFs have a blank first line before the XML decl; strip it.
    text = path.read_text(encoding='utf-8').lstrip()
    return ET.ElementTree(ET.fromstring(text))


def _entry_name(elem: ET.Element) -> str | None:
    name = elem.get('name')
    if name:
        return name
    # <include> carries name as a child element; fall back to URI basename.
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


def _ensure_robot_named(world: ET.Element) -> None:
    """Ensure the robot <include> has an explicit <name>robot</name>.

    The level system performer <ref>robot</ref> resolves against named
    entities.  Without an explicit name the include may not match.
    """
    for child in world:
        if _is_robot_include(child) and child.find('name') is None:
            name_el = ET.SubElement(child, 'name')
            name_el.text = 'robot'


def _parse_ogre_materials(path: pathlib.Path) -> dict[str, str]:
    """Parse OGRE 1.x .material file → {material_name: texture_filename}."""
    mapping: dict[str, str] = {}
    current: str | None = None
    for line in path.read_text().splitlines():
        stripped = line.strip()
        if stripped.startswith('material '):
            current = stripped.split(None, 1)[1]
        elif stripped.startswith('texture ') and current:
            mapping[current] = stripped.split(None, 1)[1]
    return mapping


def _simplify_model_collisions(models_dir: pathlib.Path) -> int:
    """Replace mesh-based collision geometry with boxes in model.sdf files.

    gz-sim's DART physics evaluates full triangle-mesh collisions every
    step, which is extremely expensive for DAE meshes.  For non-structural
    models (furniture, decorations), remove collision entirely — the robot
    navigates via gpu_lidar which still renders visuals.  Structural models
    (walls, floors, ceilings) keep their collisions untouched.
    """
    STRUCTURAL = {'wall', 'floor', 'ceiling', 'ground'}
    simplified = 0
    for sdf_path in models_dir.rglob('model.sdf'):
        model_name = sdf_path.parent.name.lower()
        if any(kw in model_name for kw in STRUCTURAL):
            continue  # keep structural collision
        try:
            tree = ET.parse(sdf_path)
        except ET.ParseError:
            continue
        root = tree.getroot()
        changed = False
        for link in root.iter('link'):
            for collision in list(link.findall('collision')):
                geom = collision.find('geometry')
                if geom is None:
                    continue
                if geom.find('mesh') is not None:
                    link.remove(collision)
                    changed = True
                    simplified += 1
        if changed:
            ET.indent(tree, space='  ')
            tree.write(sdf_path, encoding='utf-8', xml_declaration=True)
    return simplified


def _convert_material_scripts(root: ET.Element,
                              tex_map: dict[str, str]) -> int:
    """Replace OGRE 1.x <material><script> with SDF-native PBR materials.

    ogre2 (OGRE-Next) cannot load .material scripts — visuals that use them
    render solid black.  Convert to <pbr><metal><albedo_map> which ogre2
    handles natively.
    """
    converted = 0
    for mat in root.iter('material'):
        script = mat.find('script')
        if script is None:
            continue
        name_el = script.find('name')
        if name_el is None or not name_el.text:
            continue
        tex = tex_map.get(name_el.text.strip())
        if tex is None:
            continue
        mat.remove(script)
        ET.SubElement(mat, 'ambient').text = '1 1 1 1'
        ET.SubElement(mat, 'diffuse').text = '1 1 1 1'
        pbr = ET.SubElement(mat, 'pbr')
        metal = ET.SubElement(pbr, 'metal')
        ET.SubElement(metal, 'albedo_map').text = f'{TEXTURE_URI_BASE}/{tex}'
        ET.SubElement(metal, 'metalness').text = '0.0'
        ET.SubElement(metal, 'roughness').text = '0.7'
        converted += 1
    return converted


def merge() -> None:
    # Simplify mesh collisions in model files before merging worlds.
    for models_dir in [ROOT / 'models' / 'office', ROOT / 'models' / 'hospital']:
        if models_dir.is_dir():
            n = _simplify_model_collisions(models_dir)
            if n:
                print(f'simplified {n} mesh collisions in {models_dir.name}/')

    office_tree = _parse(OFFICE)
    hospital_tree = _parse(HOSPITAL)

    office_world = office_tree.getroot().find('world')
    hospital_world = hospital_tree.getroot().find('world')
    if office_world is None or hospital_world is None:
        raise SystemExit('missing <world> element')

    office_world.set('name', 'combined')

    # Replace Gazebo Classic physics block with gz-sim defaults. The
    # imported block has iters=300 and real_time_update_rate=500 which
    # pegs a Mac to ~6% RTF on this map.
    for phys in list(office_world.findall('physics')):
        office_world.remove(phys)
    phys = ET.Element('physics', {'name': '1ms', 'type': 'ignored'})
    ET.SubElement(phys, 'max_step_size').text = '0.01'
    ET.SubElement(phys, 'real_time_factor').text = '1.0'
    ET.SubElement(phys, 'real_time_update_rate').text = '100'
    office_world.insert(0, phys)

    # Inject gz-sim system plugins at the top of <world>.
    for i, (filename, name) in enumerate(GZ_SYSTEM_PLUGINS):
        plug = ET.Element('plugin', {'filename': filename, 'name': name})
        office_world.insert(1 + i, plug)

    # Ensure robot include has explicit <name> so performer can find it.
    _ensure_robot_named(office_world)

    # Replace the broken directional_light (z=-100 + attenuation range=20 →
    # nothing lit) with a proper sun. Keep original scene/ambient untouched.
    for light in list(office_world.findall('light')):
        if (light.get('type') or '').lower() == 'directional':
            office_world.remove(light)
    sun = ET.Element('light', {'name': 'sun', 'type': 'directional'})
    ET.SubElement(sun, 'pose').text = '0 0 10 0 0 0'
    ET.SubElement(sun, 'diffuse').text = '1.0 1.0 1.0 1'
    ET.SubElement(sun, 'specular').text = '0.3 0.3 0.3 1'
    ET.SubElement(sun, 'direction').text = '-0.5 0.1 -0.9'
    ET.SubElement(sun, 'cast_shadows').text = '1'
    office_world.append(sun)

    office_entries: list[dict] = []
    hospital_entries: list[dict] = []

    # Rotate every top-level office entity around world Z. The walls model
    # with the elevator sits at origin, so it's unchanged; every other model,
    # include, and light swings with it.
    for child in list(office_world):
        if child.tag not in TOP_LEVEL_COPY:
            continue
        _rotate_z_toplevel(child, OFFICE_ROTATE_Z)
        if child.tag not in MANIFEST_ENTRY_TAGS:
            continue
        if child.tag == 'light':
            ltype = (child.get('type') or '').lower()
            if ltype == 'directional':
                continue  # world-global directional light, never stash
        if _is_robot_include(child):
            continue  # robot is controlled separately
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

    # Convert OGRE 1.x material scripts to SDF PBR so ogre2 can render them.
    tex_map = _parse_ogre_materials(MATERIAL_FILE)
    converted = _convert_material_scripts(office_world, tex_map)
    print(f'converted {converted} OGRE 1.x materials → SDF PBR')

    # ── Office ceiling lights ────────────────────────────────────────
    # The office has no interior lights — the sun is blocked by the
    # ceiling, so PBR surfaces render too dark.  Add a sparse grid of
    # point lights just below the ceiling (z=2.8).  Positions are
    # computed from the rotated office model bounding box (office_entries
    # are already in the post-rotation frame).  Keep cast_shadows=0 and
    # limit to 8 lights for RTF.
    ox = [e['pose'][0] for e in office_entries]
    oy = [e['pose'][1] for e in office_entries]
    x_lo, x_hi = min(ox) + 2.0, max(ox) - 2.0
    y_lo, y_hi = min(oy) + 2.0, max(oy) - 2.0
    nx, ny = 2, 4  # 2×4 = 8 lights
    light_id = 0
    for ix in range(nx):
        lx = x_lo + ix * (x_hi - x_lo) / max(nx - 1, 1)
        for iy in range(ny):
            ly = y_lo + iy * (y_hi - y_lo) / max(ny - 1, 1)
            lt = ET.SubElement(
                office_world, 'light',
                {'name': f'office_ceiling_{light_id}', 'type': 'point'})
            ET.SubElement(lt, 'pose').text = f'{lx:.2f} {ly:.2f} 2.8 0 0 0'
            ET.SubElement(lt, 'diffuse').text = '0.85 0.85 0.8 1'
            ET.SubElement(lt, 'specular').text = '0.2 0.2 0.2 1'
            att = ET.SubElement(lt, 'attenuation')
            ET.SubElement(att, 'range').text = '18'
            ET.SubElement(att, 'linear').text = '0.1'
            ET.SubElement(att, 'quadratic').text = '0.02'
            ET.SubElement(lt, 'cast_shadows').text = '0'
            office_entries.append({
                'name': f'office_ceiling_{light_id}',
                'pose': [lx, ly, 2.8, 0, 0, 0],
            })
            light_id += 1
    print(f'added {light_id} office ceiling lights')

    # ── Overhead camera (bird's-eye view for Foxglove) ──────────────
    cx = (min(ox) + max(ox)) / 2
    cy = (min(oy) + max(oy)) / 2
    cam_model = ET.SubElement(office_world, 'model', {'name': 'overhead_cam'})
    ET.SubElement(cam_model, 'static').text = 'true'
    ET.SubElement(cam_model, 'pose').text = f'{cx:.2f} {cy:.2f} 30 0 1.5708 0'
    cam_link = ET.SubElement(cam_model, 'link', {'name': 'link'})
    cam_sensor = ET.SubElement(cam_link, 'sensor',
                               {'name': 'overhead_camera', 'type': 'camera'})
    ET.SubElement(cam_sensor, 'topic').text = '/overhead/image_raw'
    ET.SubElement(cam_sensor, 'update_rate').text = '1'
    ET.SubElement(cam_sensor, 'always_on').text = 'true'
    cam_elem = ET.SubElement(cam_sensor, 'camera')
    ET.SubElement(cam_elem, 'horizontal_fov').text = '1.57'
    img = ET.SubElement(cam_elem, 'image')
    ET.SubElement(img, 'width').text = '640'
    ET.SubElement(img, 'height').text = '480'
    ET.SubElement(img, 'format').text = 'RGB_INT8'
    clip = ET.SubElement(cam_elem, 'clip')
    ET.SubElement(clip, 'near').text = '0.5'
    ET.SubElement(clip, 'far').text = '100'
    print(f'added overhead camera at ({cx:.1f}, {cy:.1f}, 30)')

    # ── gz-sim Level System ──────────────────────────────────────────
    # LevelManager requires <performer> and <level> elements inside a
    # <plugin name="gz::sim" filename="dummy"> container.  Placing them
    # as direct children of <world> causes gz-sim to ignore them with:
    #   "Could not find a plugin tag with name gz::sim"
    def _bbox(entries: list[dict], pad: float = 15.0) -> tuple[list[float], list[float]]:
        xs = [e['pose'][0] for e in entries]
        ys = [e['pose'][1] for e in entries]
        center = [(min(xs)+max(xs))/2, (min(ys)+max(ys))/2, 1.5]
        size = [max(xs)-min(xs)+pad*2, max(ys)-min(ys)+pad*2, 15.0]
        return center, size

    level_plugin = ET.SubElement(
        office_world, 'plugin', {'name': 'gz::sim', 'filename': 'dummy'})

    # Performer: the robot triggers level loading.
    perf = ET.SubElement(level_plugin, 'performer', {'name': 'robot_perf'})
    ET.SubElement(perf, 'ref').text = 'robot'
    geom = ET.SubElement(perf, 'geometry')
    box = ET.SubElement(geom, 'box')
    ET.SubElement(box, 'size').text = '2 2 2'

    for label, entries in [('office', office_entries),
                           ('hospital', hospital_entries)]:
        if not entries:
            continue
        center, size = _bbox(entries)
        lvl = ET.SubElement(level_plugin, 'level', {'name': f'{label}_level'})
        ET.SubElement(lvl, 'pose').text = _format_pose(center + [0, 0, 0])
        lg = ET.SubElement(lvl, 'geometry')
        lb = ET.SubElement(lg, 'box')
        ET.SubElement(lb, 'size').text = f'{size[0]:.1f} {size[1]:.1f} {size[2]:.1f}'
        ET.SubElement(lvl, 'buffer').text = '5'
        for e in entries:
            ET.SubElement(lvl, 'ref').text = e['name']
        print(f'level {label}: center=({center[0]:.1f},{center[1]:.1f}) '
              f'size=({size[0]:.1f}×{size[1]:.1f}), {len(entries)} refs')

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
