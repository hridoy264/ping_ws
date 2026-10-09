#!/usr/bin/env python3
"""Write exactly N physical ball models, or append them to a copy of an SDF world.

Coordinates describe an explicit empty AABB in world metres. No CAD fit,
settling, feeding, or runtime validation is implied by a generated file.
"""

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pingpong_launcher_sim.inventory import (  # noqa: E402
    AABB, BallSpec, append_to_world, inventory_sdf, pack_balls,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bounds', type=float, nargs=6, required=True,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'))
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--radius', type=float, default=0.020)
    parser.add_argument('--mass', type=float, default=0.0027)
    parser.add_argument('--surface-gap', type=float, default=0.001)
    parser.add_argument('--wall-clearance', type=float, default=0.001)
    parser.add_argument('--jitter', type=float, default=0.0,
                        help='Maximum displacement per axis, metres; spacing includes clearance')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--prefix', default='inventory_ball')
    parser.add_argument('--input-world', type=Path)
    parser.add_argument('--world-name', help='Required only when input contains multiple worlds')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--force', action='store_true', help='Replace an existing output file')
    args = parser.parse_args(argv)
    try:
        if args.input_world and args.input_world.resolve() == args.output.resolve():
            raise ValueError('Output must differ from input; original world is preserved')
        if args.world_name and not args.input_world:
            raise ValueError('--world-name requires --input-world')
        if args.output.exists() and not args.force:
            raise ValueError('Output exists; choose another path or pass --force')
        packing = pack_balls(AABB(tuple(args.bounds[:3]), tuple(args.bounds[3:])),
                             args.count, BallSpec(args.radius, args.mass),
                             surface_gap=args.surface_gap, wall_clearance=args.wall_clearance,
                             jitter=args.jitter, seed=args.seed, prefix=args.prefix)
        if args.input_world:
            xml_parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
            root = append_to_world(ET.parse(args.input_world, parser=xml_parser).getroot(),
                                   packing, world_name=args.world_name)
        else:
            root = inventory_sdf(packing)
        ET.indent(root, space='  ')
        encoded = ET.tostring(root, encoding='utf-8', xml_declaration=True) + b'\n'
        with args.output.open('wb' if args.force else 'xb') as output:
            output.write(encoded)
    except (ValueError, OSError, ET.ParseError) as exc:
        parser.error(str(exc))
    print(json.dumps({'output': str(args.output.resolve()), 'count': len(packing.balls),
                      'grid_capacity': packing.capacity, 'grid_shape': packing.grid_shape,
                      'pitch_m': packing.pitch, 'total_mass_kg': args.count * args.mass,
                      'bounds_m': args.bounds, 'seed': args.seed,
                      'runtime_validated': False, 'hopper_fit_validated': False,
                      'format': 'world' if args.input_world else 'model_fragment'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
