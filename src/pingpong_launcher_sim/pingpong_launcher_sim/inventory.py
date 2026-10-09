"""Finite ball inventory, initial placement, and SDF generation (stdlib only).

This module neither moves bodies nor detects contact. A ledger records external
observations; initial packing only establishes sphere-to-sphere/AABB clearance.
"""

from collections import Counter
from dataclasses import dataclass
from enum import Enum
import copy
import math
import random
import re
import xml.etree.ElementTree as ET


def _finite(value, name, minimum=0.0, strict=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or
            (value <= minimum if strict else value < minimum)):
        raise ValueError(f'{name} must be finite and {">" if strict else ">="} {minimum}')
    return float(value)


@dataclass(frozen=True)
class BallSpec:
    radius: float = 0.020
    mass: float = 0.0027
    friction: float = 0.30
    restitution: float = 0.80

    def __post_init__(self):
        _finite(self.radius, 'radius', strict=True)
        _finite(self.mass, 'mass', strict=True)
        _finite(self.friction, 'friction')
        _finite(self.restitution, 'restitution')
        if self.restitution > 1:
            raise ValueError('restitution must be <= 1')

    @property
    def inertia(self):
        """Thin spherical shell moment about any axis, in kg m²."""
        return (2.0 / 3.0) * self.mass * self.radius ** 2


@dataclass(frozen=True)
class AABB:
    """Available volume in metres, excluding the physical enclosure walls."""
    minimum: tuple
    maximum: tuple

    def __post_init__(self):
        if len(self.minimum) != 3 or len(self.maximum) != 3:
            raise ValueError('AABB requires three minimum and three maximum coordinates')
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and
                   math.isfinite(v) for v in (*self.minimum, *self.maximum)):
            raise ValueError('AABB coordinates must be finite numbers')
        if not all(hi > lo for lo, hi in zip(self.minimum, self.maximum)):
            raise ValueError('Every AABB maximum must exceed its minimum')
        object.__setattr__(self, 'minimum', tuple(self.minimum))
        object.__setattr__(self, 'maximum', tuple(self.maximum))


@dataclass(frozen=True)
class BallPlacement:
    name: str
    position: tuple


@dataclass(frozen=True)
class Packing:
    balls: tuple
    bounds: AABB
    spec: BallSpec
    grid_shape: tuple
    pitch: float
    surface_gap: float
    wall_clearance: float
    jitter: float
    seed: int

    @property
    def capacity(self):
        return math.prod(self.grid_shape)


def pack_balls(bounds, count=100, spec=None, *, surface_gap=0.001,
               wall_clearance=0.001, jitter=0.0, seed=0, prefix='inventory_ball'):
    """Return a deterministic centred grid, filled bottom layer first.

    Jitter is bounded independently on every axis. Grid pitch includes twice
    that bound, so even neighbouring spheres moving toward one another retain
    the requested surface gap. Capacity is conservative rectangular packing,
    not a prediction of settled hopper capacity.
    """
    if not isinstance(bounds, AABB):
        raise ValueError('bounds must be an AABB')
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError('count must be a positive integer')
    if not isinstance(seed, int) or isinstance(seed, bool):
        raise ValueError('seed must be an integer')
    if not isinstance(prefix, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', prefix):
        raise ValueError('prefix must be a simple SDF entity name')
    spec = spec or BallSpec()
    surface_gap = _finite(surface_gap, 'surface_gap')
    wall_clearance = _finite(wall_clearance, 'wall_clearance')
    jitter = _finite(jitter, 'jitter')
    pitch = 2 * spec.radius + surface_gap + 2 * jitter
    margin = spec.radius + wall_clearance + jitter
    spans = tuple(hi - lo for lo, hi in zip(bounds.minimum, bounds.maximum))
    shape = tuple(0 if span < 2 * margin - 1e-12 else
                  max(0, math.floor((span - 2 * margin) / pitch + 1e-12) + 1)
                  for span in spans)
    capacity = math.prod(shape)
    if capacity < count:
        raise ValueError(f'AABB holds {capacity} grid positions, fewer than requested {count}; '
                         'increase the available bounds or load above the opening')
    starts = tuple((lo + hi - (n - 1) * pitch) / 2
                   for lo, hi, n in zip(bounds.minimum, bounds.maximum, shape))
    rng = random.Random(seed)
    balls = []
    for iz in range(shape[2]):
        for iy in range(shape[1]):
            for ix in range(shape[0]):
                position = tuple(start + index * pitch + rng.uniform(-jitter, jitter)
                                 for start, index in zip(starts, (ix, iy, iz)))
                balls.append(BallPlacement(f'{prefix}_{len(balls) + 1:03d}', position))
                if len(balls) == count:
                    return Packing(tuple(balls), bounds, spec, shape, pitch,
                                   surface_gap, wall_clearance, jitter, seed)
    raise AssertionError('packing count and grid capacity disagreed')


def _node(parent, tag, value=None, **attributes):
    result = ET.SubElement(parent, tag, attributes)
    if value is not None:
        result.text = str(value)
    return result


def ball_model(placement, spec=None):
    """Create one free, persistent physical sphere; no expiry/spawn plugin."""
    spec = spec or BallSpec()
    model = ET.Element('model', name=placement.name)
    _node(model, 'static', 'false')
    _node(model, 'pose', ' '.join(format(v, '.17g') for v in (*placement.position, 0, 0, 0)))
    link = _node(model, 'link', name='ball_link')
    _node(link, 'gravity', 'true')
    inertia = _node(link, 'inertial')
    _node(inertia, 'mass', format(spec.mass, '.17g'))
    tensor = _node(inertia, 'inertia')
    for axis in ('ixx', 'iyy', 'izz'):
        _node(tensor, axis, format(spec.inertia, '.17g'))
    for axis in ('ixy', 'ixz', 'iyz'):
        _node(tensor, axis, '0')
    collision = _node(link, 'collision', name='ball_collision')
    sphere = _node(_node(collision, 'geometry'), 'sphere')
    _node(sphere, 'radius', format(spec.radius, '.17g'))
    surface = _node(collision, 'surface')
    friction = _node(_node(surface, 'friction'), 'ode')
    _node(friction, 'mu', spec.friction)
    _node(friction, 'mu2', spec.friction)
    bounce = _node(surface, 'bounce')
    _node(bounce, 'restitution_coefficient', spec.restitution)
    _node(bounce, 'threshold', '0.01')
    visual = _node(link, 'visual', name='ball_visual')
    sphere = _node(_node(visual, 'geometry'), 'sphere')
    _node(sphere, 'radius', format(spec.radius, '.17g'))
    material = _node(visual, 'material')
    _node(material, 'ambient', '0.96 0.96 0.92 1')
    _node(material, 'diffuse', '0.96 0.96 0.92 1')
    return model


def append_to_world(root, packing, *, world_name=None):
    """Copy an SDF document and add the inventory without altering input XML.

    Existing plugins, models, and world names are preserved. Duplicate direct
    entity names are rejected. Existing launch/spawn plugins are not disabled;
    their presence must be dealt with by the simulator integration.
    """
    if root.tag != 'sdf':
        raise ValueError('Input root must be <sdf>')
    result = copy.deepcopy(root)
    worlds = result.findall('world')
    if world_name is not None:
        worlds = [world for world in worlds if world.get('name') == world_name]
    if len(worlds) != 1:
        raise ValueError('Select exactly one SDF world with world_name')
    world = worlds[0]
    existing = {child.get('name') for child in world if child.get('name')}
    # Includes can override model names explicitly. Their unresolved model URIs
    # are retained; resource-resolution checks remain the caller's responsibility.
    existing.update(node.text for node in world.findall('include/name') if node.text)
    names = [ball.name for ball in packing.balls]
    if len(set(names)) != len(names) or existing.intersection(names):
        raise ValueError('Ball names are duplicated or conflict with existing world entities')
    world.append(ET.Comment(f' {len(names)} finite inventory balls; seed={packing.seed}; '
                            'AABB packing is not hopper/contact validation '))
    for ball in packing.balls:
        world.append(ball_model(ball, packing.spec))
    return result


def inventory_sdf(packing):
    """Return an SDF model fragment; insert these models into a physical world."""
    root = ET.Element('sdf', version='1.8')
    for ball in packing.balls:
        root.append(ball_model(ball, packing.spec))
    return root


class Phase(str, Enum):
    INITIALIZED = 'initialized'
    HOPPER = 'hopper'
    FEEDER = 'feeder'
    PIPE = 'pipe'
    HEAD = 'head'
    FLIGHT = 'flight'
    COLLECTED = 'collected'
    OUT_OF_BOUNDS = 'out_of_bounds'


@dataclass(frozen=True)
class Observation:
    phase: Phase = Phase.INITIALIZED
    sim_time: float | None = None


class BallLedger:
    """Fixed identity registry; phase changes require an external observation.

    Phase is the last recorded location, not inferred success. ``audit`` compares
    that registry to live Gazebo names, exposing missing, extra and duplicate
    observations without quietly creating/removing registered balls.
    """

    def __init__(self, names):
        names = tuple(names)
        if not names or any(not isinstance(name, str) or not name for name in names):
            raise ValueError('A nonempty inventory of nonempty string names is required')
        if len(set(names)) != len(names):
            raise ValueError('Inventory identities must be unique')
        self._records = dict.fromkeys(names, Observation())

    @property
    def names(self):
        return tuple(self._records)

    def observe(self, name, phase, sim_time):
        if name not in self._records:
            raise KeyError(f'Unregistered ball: {name}')
        phase = Phase(phase)
        sim_time = _finite(sim_time, 'sim_time')
        previous = self._records[name]
        if previous.sim_time is not None and sim_time < previous.sim_time:
            raise ValueError('Observation time moved backward; reset the ledger explicitly')
        self._records[name] = Observation(phase, sim_time)
        return previous.phase != phase

    def reset(self):
        self._records = dict.fromkeys(self._records, Observation())

    def snapshot(self):
        return dict(self._records)

    def counts(self):
        counts = Counter(record.phase.value for record in self._records.values())
        return {phase.value: counts[phase.value] for phase in Phase}

    def audit(self, observed_names):
        seen = Counter(observed_names)
        expected = set(self._records)
        observed = set(seen)
        missing = sorted(expected - observed)
        unexpected = sorted(observed - expected)
        duplicates = sorted(name for name, count in seen.items() if count != 1)
        return {'expected_count': len(expected), 'observed_unique_count': len(observed),
                'missing': missing, 'unexpected': unexpected, 'duplicates': duplicates,
                'all_accounted_for': not (missing or unexpected or duplicates),
                'phase_counts': self.counts()}
