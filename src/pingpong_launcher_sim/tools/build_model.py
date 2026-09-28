#!/usr/bin/env python3
"""Generate the R5 Fortress model from preserved Fusion STL assets (stdlib only).

CAD STL coordinates are millimetres. Every output vertex is physically converted
into metres in its owning link frame: SDF mesh scales are therefore 1 1 1.
Run from any directory in the original PingPong checkout. Generated assets ship
with the package; installed packages do not need the CAD source tree.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[2]
MODEL = PACKAGE / 'models' / 'pingpong_launcher'
IDENTITY = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
HEAD_CAD = ((0, 0, 1), (1, 0, 0), (0, 1, 0))
HEAD = (-.033, 0, .23)
YAW = (-.033, 0, .084)
COLORS = {'body': '.78 .83 .88 1', 'blue': '.07 .28 .48 1',
          'dark': '.09 .11 .14 1', 'metal': '.55 .58 .61 1',
          'orange': '.94 .40 .09 1', 'rubber': '.04 .045 .05 1'}


def fmt(values):
    if isinstance(values, (str, int, float)):
        return str(values)
    return ' '.join(f'{v:.12g}' for v in values)


def elem(parent, tag, value=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if value is not None:
        node.text = fmt(value)
    return node


def pose(parent, xyz=(0, 0, 0), rpy=(0, 0, 0), relative_to=None):
    attrs = {'relative_to': relative_to} if relative_to else {}
    return elem(parent, 'pose', (*xyz, *rpy), **attrs)


def mul(matrix, vec):
    return tuple(sum(row[j] * vec[j] for j in range(3)) for row in matrix)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def neg(a):
    return tuple(-v for v in a)


def rpy_from_matrix(m):
    pitch = math.asin(max(-1, min(1, -m[2][0])))
    if abs(math.cos(pitch)) > 1e-9:
        return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])
    return 0, pitch, math.atan2(-m[0][1], m[1][1])


def station(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    # Columns: outward radial, launch +X, original CAD motor-axis direction.
    return ((0, 1, 0), (c, 0, s), (s, 0, -c))


def convert_stl(source, target, matrix=IDENTITY, shift=(0, 0, 0)):
    data = source.read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    if len(data) != 84 + 50 * count:
        raise ValueError(f'Expected binary STL: {source}')
    result = bytearray(b'PingPong R5; metres; local link frame'.ljust(80, b'\0'))
    result.extend(struct.pack('<I', count))
    low, high = [float('inf')] * 3, [-float('inf')] * 3
    for i in range(count):
        raw = struct.unpack_from('<12fH', data, 84 + 50 * i)
        normal = mul(matrix, raw[:3])
        verts = []
        for j in (3, 6, 9):
            v = add(mul(matrix, [value / 1000 for value in raw[j:j + 3]]), shift)
            if not all(math.isfinite(x) for x in v):
                raise ValueError(f'Nonfinite vertex in {source}')
            for k in range(3):
                low[k], high[k] = min(low[k], v[k]), max(high[k], v[k])
            verts.extend(v)
        result.extend(struct.pack('<12fH', *normal, *verts, raw[-1]))
    target.write_bytes(result)
    return {'source': str(source.relative_to(REPO)),
            'source_sha256': hashlib.sha256(data).hexdigest(),
            'output': str(target.relative_to(MODEL)), 'triangles': count,
            'source_unit': 'millimetres', 'output_unit': 'metres',
            'rotation': matrix, 'translation_metres': shift,
            'bounds_metres': [low, high]}


def material(parent, color):
    m = elem(parent, 'material')
    elem(m, 'ambient', COLORS.get(color, color))
    elem(m, 'diffuse', COLORS.get(color, color))
    elem(m, 'specular', '.18 .18 .18 1')


def mesh_visual(link, name, filename, color='body'):
    v = elem(link, 'visual', name=name)
    mesh = elem(elem(v, 'geometry'), 'mesh')
    elem(mesh, 'uri', f'model://pingpong_launcher/meshes/{filename}')
    elem(mesh, 'scale', '1 1 1')
    material(v, color)


def primitive(parent, name, kind, size, xyz=(0, 0, 0), rpy=(0, 0, 0), color=None):
    node = elem(parent, 'visual' if color else 'collision', name=name)
    pose(node, xyz, rpy)
    shape = elem(elem(node, 'geometry'), kind)
    if kind == 'box':
        elem(shape, 'size', size)
    elif kind == 'cylinder':
        elem(shape, 'radius', size[0]); elem(shape, 'length', size[1])
    elif kind == 'sphere':
        elem(shape, 'radius', size[0])
    if color:
        material(node, color)
    else:
        surface = elem(node, 'surface')
        friction = elem(elem(surface, 'friction'), 'ode')
        elem(friction, 'mu', .6); elem(friction, 'mu2', .6)
        bounce = elem(surface, 'bounce')
        elem(bounce, 'restitution_coefficient', .1)
        elem(bounce, 'threshold', .1)
    return node


def ring_collision(link, name, axis, center, radius, thickness, length, segments=16):
    # Disjoint/overlapping convex panels preserve the central opening. Axial X
    # panels use local (axial,tangent,radial); axial Z panels (radial,tangent,Z).
    width = 2 * (radius + thickness / 2) * math.tan(math.pi / segments)
    for i in range(segments):
        a = 2 * math.pi * i / segments
        c, s = math.cos(a), math.sin(a)
        if axis == 'X':
            xyz = add(center, (0, radius * c, radius * s))
            # local Y is tangent, local Z points inward/outward radially.
            rpy = (a - math.pi / 2, 0, 0)
            size = (length, width, thickness)
        else:
            xyz = add(center, (radius * c, radius * s, 0))
            rpy = (0, 0, a)
            size = (thickness, width, length)
        primitive(link, f'{name}_{i:02d}', 'box', size, xyz, rpy)


def box_inertia(mass, size):
    x, y, z = size
    return (mass * (y*y+z*z)/12, mass * (x*x+z*z)/12,
            mass * (x*x+y*y)/12, 0, 0, 0)


def cylinder_inertia(mass, radius, length, axis=(0, 0, 1)):
    axial = mass * radius * radius / 2
    transverse = mass * (3 * radius * radius + length * length) / 12
    m = [[(transverse if i == j else 0) + (axial-transverse)*axis[i]*axis[j]
          for j in range(3)] for i in range(3)]
    return m[0][0], m[1][1], m[2][2], m[0][1], m[0][2], m[1][2]


def link(model, name, xyz, mass, inertia, com=(0, 0, 0), relative_to=None):
    node = elem(model, 'link', name=name)
    pose(node, xyz, relative_to=relative_to)
    elem(node, 'self_collide', 'false')
    inertial = elem(node, 'inertial')
    pose(inertial, com)
    elem(inertial, 'mass', mass)
    tensor = elem(inertial, 'inertia')
    for key, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), inertia):
        elem(tensor, key, f'{value:.12g}')
    return node


def joint(model, name, parent, child, axis=None, limits=None, velocity=1, effort=1,
          damping=.01):
    node = elem(model, 'joint', name=name, type='revolute' if axis else 'fixed')
    elem(node, 'parent', parent); elem(node, 'child', child)
    # Default joint pose is child-link frame; all origins are at real pivot axes.
    pose(node, relative_to=child)
    if axis:
        ax = elem(node, 'axis'); elem(ax, 'xyz', axis)
        lim = elem(ax, 'limit')
        if limits:
            elem(lim, 'lower', limits[0]); elem(lim, 'upper', limits[1])
        else:
            elem(lim, 'lower', -1e16); elem(lim, 'upper', 1e16)
        elem(lim, 'velocity', velocity); elem(lim, 'effort', effort)
        elem(elem(ax, 'dynamics'), 'damping', damping)
    return node


def controller(model, joint_name, topic, position=False, speed=1):
    suffix = 'joint-position-controller-system' if position else 'joint-controller-system'
    cls = 'JointPositionController' if position else 'JointController'
    p = elem(model, 'plugin', filename='ignition-gazebo-' + suffix,
             name='ignition::gazebo::systems::' + cls)
    elem(p, 'joint_name', joint_name); elem(p, 'topic', topic)
    if position:
        # Fortress's ideal velocity-command mode avoids uncertain torque PID
        # claims; cmd_max caps output speed. Physical motor dynamics remain TBD.
        elem(p, 'use_velocity_commands', 'true')
        elem(p, 'cmd_max', speed); elem(p, 'cmd_min', -speed)
        elem(p, 'initial_position', 0)
    else:
        elem(p, 'use_force_commands', 'false'); elem(p, 'initial_velocity', 0)


def generate(source_root=REPO):
    global REPO
    REPO = source_root.resolve()
    (MODEL / 'meshes').mkdir(parents=True, exist_ok=True)
    sdf = ET.Element('sdf', version='1.8')
    model = elem(sdf, 'model', name='pingpong_launcher', canonical_link='base_link')
    elem(model, 'static', 'false'); elem(model, 'self_collide', 'false')
    base = link(model, 'base_link', (0, 0, 0), 2.8,
                box_inertia(2.8, (.58, .25, .59)), (-.19, 0, .22))
    yaw = link(model, 'yaw_link', YAW, .62,
               box_inertia(.62, (.24, .345, .20)), (0, .012, .035))
    head = link(model, 'head_link', (0, 0, .146), .93,
                box_inertia(.93, (.16, .24, .24)), (.005, 0, 0), 'yaw_link')
    feeder = link(model, 'feeder_link', (-.285, 0, .3506), .09,
                  cylinder_inertia(.09, .086, .0415), (0, 0, .02075))
    joint(model, 'world_fixed', 'world', 'base_link')
    joint(model, 'yaw_joint', 'base_link', 'yaw_link', (0, 0, 1),
          (-math.radians(25), math.radians(25)), .6, 2.0)
    joint(model, 'pitch_joint', 'yaw_link', 'head_link', (0, 1, 0),
          (-math.radians(20), math.radians(10)), .45, 1.8)
    joint(model, 'feeder_joint', 'base_link', 'feeder_link', (0, 0, 1),
          velocity=2, effort=.5)
    manifest = []

    def cad(link_node, name, relative, matrix=IDENTITY, shift=(0, 0, 0), color='body'):
        filename = name + '.stl'
        item = convert_stl(REPO / relative, MODEL / 'meshes' / filename, matrix, shift)
        item['link'] = link_node.attrib['name']
        manifest.append(item)
        mesh_visual(link_node, name, filename, color)

    r4 = 'fusion/AimingR4/STL/'
    r3 = 'fusion/FeederR3/STL/'
    r2 = 'fusion/ThrowerR2/STL/'
    for path in sorted((REPO / r4).glob('*.stl')):
        name = path.stem
        code = name.split('_')[0]
        if code == 'A01':
            continue
        if code == 'A02':
            cad(head, name, str(path.relative_to(REPO)), HEAD_CAD, (.033, 0, 0))
        elif code in ('A04', 'A11', 'A14', 'A15'):
            cad(head, name, str(path.relative_to(REPO)), shift=neg(HEAD),
                color='orange' if code == 'A11' else 'blue')
        elif code in ('A07', 'A09', 'A10', 'A12', 'A13'):
            cad(yaw, name, str(path.relative_to(REPO)), shift=neg(YAW),
                color='orange' if code in ('A07', 'A12') else 'blue')
        else:
            cad(base, name, str(path.relative_to(REPO)),
                color='orange' if code == 'A08' else 'blue')
    cad(head, 'r5_front_cover', 'fusion/AimingR5/A01_Shallow_front_90mm.stl',
        HEAD_CAD, (.033, 0, 0))
    cad(head, 'rear_housing', r2 + 'P01_Rear_housing_-_male_thread.stl',
        HEAD_CAD, (.033, 0, 0))
    for path in sorted((REPO / r3).glob('*.stl')):
        if path.name.startswith(('F01_', 'F02_', 'F04_', 'F07_', 'F08_')):
            cad(base, path.stem, str(path.relative_to(REPO)), shift=(-.06, 0, 0),
                color='body' if path.name.startswith(('F01_', 'F02_', 'F04_')) else 'blue')
    cad(feeder, 'feeder_rotor', r3 + 'F03_Four-pocket_rotor_-_90deg_index.stl',
        shift=(.225, 0, -.3506), color='orange')
    for n, deg in enumerate((90, 210, 330), 1):
        a = math.radians(deg)
        center = (.033, .052 * math.cos(a), .052 * math.sin(a))
        axis = (0, -math.sin(a), math.cos(a))
        rotation = station(deg)
        wheel = link(model, f'wheel_{n}_link', center, .035,
                     cylinder_inertia(.035, .0325, .018, axis), relative_to='head_link')
        joint(model, f'wheel_{n}_joint', 'head_link', f'wheel_{n}_link', axis,
              velocity=1200, effort=.25, damping=0)
        cad(wheel, f'wheel_{n}_core', r2 + 'P04_Wheel_core_-_6.3mm_adapter_bore_-_print_three.stl',
            rotation, color='orange')
        cad(head, f'motor_{n}_bracket', r2 + 'P03_Ribbed_A2212_mount_-_print_three.stl',
            rotation, center, 'blue')
        orient = rpy_from_matrix(rotation)
        # Tyre visual envelope and collision. The core remains separately visible
        # on both outer faces; this cylinder represents the rubber contact tread.
        primitive(wheel, 'rubber_tread', 'cylinder', (.0325, .0156), rpy=orient, color='rubber')
        primitive(wheel, 'wheel_contact', 'cylinder', (.0325, .018), rpy=orient)
        marker = add(mul(rotation, (.024, 0, .0092)), (0, 0, 0))
        primitive(wheel, 'rotation_marker', 'box', (.013, .002, .0005), marker,
                  orient, 'body')
        motor_center = add(center, mul(rotation, (0, 0, -.036)))
        primitive(head, f'motor_{n}_envelope', 'cylinder', (.014, .03),
                  motor_center, orient, 'metal')
        primitive(head, f'motor_{n}_body_collision', 'cylinder', (.014, .03),
                  motor_center, orient)
        controller(model, f'wheel_{n}_joint', f'/pingpong/wheel_{n}_cmd')

    # Purchased components are explicitly simplified envelopes, not exported CAD.
    primitive(base, 'yaw_stepper', 'box', (.042, .042, .042), (.057, 0, .0295), color='dark')
    primitive(base, 'feeder_stepper', 'box', (.042, .042, .042), (-.285, 0, .323), color='dark')
    primitive(yaw, 'pitch_servo', 'box', (.0197, .037, .0407),
              (0, .1755, .06815), color='dark')
    primitive(yaw, 'yaw_shaft', 'cylinder', (.004, .092), (0, 0, -.034), color='metal')
    primitive(head, 'pitch_shaft', 'cylinder', (.004, .315), rpy=(math.pi/2, 0, 0), color='metal')

    # Broad, stable primitives; threaded detailed meshes are visual only.
    primitive(base, 'base_foot', 'box', (.24, .24, .008), (-.033, 0, .004))
    primitive(base, 'feeder_base', 'box', (.23, .244, .008), (-.285, 0, .004))
    primitive(base, 'yaw_bearing_tower', 'cylinder', (.016, .047), (-.033, 0, .0315))
    for sign in (-1, 1):
        primitive(base, f'feeder_post_{sign}', 'box', (.032, .02, .336),
                  (-.285, sign*.098, .176))
        primitive(yaw, f'fork_{sign}', 'box', (.068, .012, .149),
                  (0, sign*.136, .0855))
    primitive(yaw, 'turntable', 'cylinder', (.118, .008), (0, 0, 0))
    ring_collision(base, 'feeder_pan_wall', 'Z', (-.285, 0, .37135), .092, .008, .0427)
    # Pan floor and cover use rings as approximations; no claim of singulation contact.
    ring_collision(base, 'pan_floor', 'Z', (-.285, 0, .347), .078, .036, .006)
    ring_collision(base, 'feeder_lid', 'Z', (-.285, 0, .3957), .077, .038, .006)
    primitive(feeder, 'rotor_hub', 'cylinder', (.014, .0415), (0, 0, .02075))
    for i in range(4):
        a = i*math.pi/2
        ring_collision(feeder, f'pocket_{i}', 'Z', (.06*math.cos(a), .06*math.sin(a), .02075),
                       .0235, .003, .0415, 12)
    # Rear cylindrical shell and rear annular mounting plate leave the feed bore open.
    ring_collision(head, 'rear_shell', 'X', (-.0005, 0, 0), .1085, .003, .063)
    ring_collision(head, 'rear_plate', 'X', (-.031, 0, 0), .065, .078, .006)
    ring_collision(head, 'front_collar', 'X', (.043, 0, 0), .1085, .003, .02)
    # Taper approximation: short rings with monotonically decreasing radius.
    for i in range(8):
        t = (i+.5)/8
        x = .053 + .035*t
        radius = .109 + (.0465-.109)*t
        ring_collision(head, f'cover_taper_{i}', 'X', (x, 0, 0), radius, .003,
                       .035/8, 20)
    ring_collision(head, 'cover_lip', 'X', (.0865, 0, 0), .0465, .003, .003, 20)
    # Smooth-bore inlet proxy: the inlet remains open but hose dynamics are omitted.
    ring_collision(head, 'head_inlet', 'X', (-.036, 0, 0), .023, .003, .072)

    controller(model, 'yaw_joint', '/pingpong/yaw_cmd', True, .6)
    controller(model, 'pitch_joint', '/pingpong/pitch_cmd', True, .45)
    controller(model, 'feeder_joint', '/pingpong/feeder_cmd', True, 2)
    publisher = elem(model, 'plugin', filename='ignition-gazebo-joint-state-publisher-system',
                     name='ignition::gazebo::systems::JointStatePublisher')
    elem(publisher, 'topic', '/pingpong/joint_states')
    elem(publisher, 'update_rate', 100)
    for name in ('yaw_joint', 'pitch_joint', 'feeder_joint', 'wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint'):
        elem(publisher, 'joint_name', name)
    ball = elem(model, 'plugin', filename='libpingpong_ball_launcher.so', name='pingpong::BallLauncher')
    elem(ball, 'head_link', 'head_link')
    for i in range(1, 4):
        elem(ball, 'wheel_joint', f'wheel_{i}_joint')
    for name, value in {'muzzle_pose': '.113 0 0 0 0 0', 'wheel_radius': .0325,
                        'efficiency': .75, 'ball_radius': .02, 'ball_mass': .0027,
                        'max_balls': 30, 'ball_lifetime': 15, 'min_shot_interval': .25,
                        'drag_coefficient': .47, 'air_density': 1.225,
                        'fire_topic': '/pingpong/fire', 'shot_count_topic': '/pingpong/shot_count',
                        'status_topic': '/pingpong/ball_status'}.items():
        elem(ball, name, value)
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(MODEL / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (MODEL / 'mesh_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    config = ET.Element('model')
    elem(config, 'name', 'PingPong Launcher R5'); elem(config, 'version', '1.0')
    elem(config, 'sdf', 'model.sdf', version='1.8')
    author = elem(config, 'author'); elem(author, 'name', 'PingPong project')
    elem(config, 'description', 'R5 CAD-derived six-joint prototype. Estimated dynamics; abstract ball launcher; no hose or singulation simulation.')
    ET.indent(config, space='  ')
    ET.ElementTree(config).write(MODEL / 'model.config', encoding='utf-8', xml_declaration=True)
    print(f'Generated {len(manifest)} CAD visual meshes, 7 links, 6 actuated joints in {MODEL}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=REPO,
                        help='PingPong repository containing original fusion/ CAD assets')
    args = parser.parse_args()
    generate(args.source_root)
