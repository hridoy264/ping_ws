# Finite 100-ball inventory support

This addition generates 100 uniquely named physical sphere models and provides
an identity-conserving observation ledger. It has no automatic expiration and
does not remove an old ball to make room for another. Nominal balls are 40 mm in
diameter, 2.7 g each (270 g total), with thin hollow sphere inertia
`I = 2mr²/3 = 7.2e-7 kg m²` about each principal axis.

It is infrastructure for the new compact robot. It does **not** demonstrate
hopper capacity, settling, successful feeding or launch. The existing R5
`BallLauncher` plugin still creates new muzzle balls when asked to fire. Do not
combine its fire action with this inventory and report conservation of 100
balls. Simulator integration must launch an existing inventory ball when that
ball reaches the head.

## Generate a copied world

Run from the package directory. This example box is deliberately an arbitrary
300 mm cube above the origin; replace it with an empty, verified region in the
assembled robot's world frame before loading balls into a real hopper.

```bash
python3 tools/generate_ball_inventory.py \
  --input-world worlds/training.sdf \
  --output /tmp/training_inventory.sdf \
  --bounds 0 0 1.2 0.3 0.3 1.5 \
  --count 100 --seed 42 --jitter 0.001
```

Bounds are `xmin ymin zmin xmax ymax zmax`, in **world metres**, describing the
available space inside walls or above the hopper opening. The tool does not
infer geometry from these bounds. A loading region above the opening must also
fit horizontally through that opening; its floor is not a physical support.
Do not use the full enclosing box of a tapered hopper as if every point were
empty interior space.

The default sphere surface gap and wall clearance are each 1 mm. Packing is a
deterministic rectangular grid, filled from the bottom. Per-axis jitter is
bounded and grid pitch includes twice that jitter, preserving the requested
sphere gap for every seed. If the requested count does not fit, generation
fails with the conservative grid capacity. This does not estimate random
settled packing density.

The tool preserves the selected world's name and existing contents, rejects
duplicate direct entity names, and refuses to overwrite its input. It refuses
an existing output unless `--force` is supplied. For documents containing
multiple worlds, select one with `--world-name`. Model URIs remain unresolved;
existing included model names and geometry still require runtime verification.
Without `--input-world`, output is an SDF **model fragment** containing multiple
models to insert into a world, not a complete runnable world.

Radius, mass, gap and clearance are configurable. Contact values default to the
same uncalibrated friction/restitution estimates as the previous launcher;
`BallSpec` permits explicit replacement. Rigid spheres and these coefficients
do not represent rubber compression or establish accurate physical bouncing.

## Track the same balls

```python
from pingpong_launcher_sim.inventory import BallLedger, Phase

names = [f'inventory_ball_{i:03d}' for i in range(1, 101)]
ledger = BallLedger(names)
# Called only after a sensor or geometry observer locates this same entity:
ledger.observe('inventory_ball_001', Phase.PIPE, sim_time=4.5)
report = ledger.audit(live_inventory_entity_names)
```

States are `initialized`, `hopper`, `feeder`, `pipe`, `head`, `flight`,
`collected`, and `out_of_bounds`. State changes record an external observation;
they never move a ball or prove a feed command succeeded. The initial state
deliberately does not claim the ball has settled inside the hopper. Observed
reverse travel can be recorded rather than rejected. Per-ball time must not go
backward; call `reset()` explicitly when resetting simulation.

The registry always retains its original identities. `audit` exposes missing,
unexpected and duplicate live names without fabricating replacement balls.
Phase counts describe last recorded locations; only a live-name audit can
confirm entity conservation. Callers must filter Gazebo model names to the
inventory scope before passing them to `audit`.

## Checks and remaining integration

```bash
python3 -m unittest discover -s test -p 'test_ball_inventory.py' -v
```

Tests check pairwise clearance for 100 spheres under three seeds, bounds and
capacity, seeded reproducibility, hollow sphere mass/inertia, safe world
copying, CLI overwrite refusal and registry conservation/missing detection.
They do not run Gazebo. This additive change does not edit controller, launcher
plugin, existing world, model generator or CMake test registration.

Next integrate the completed CAD collision corridor, the observation/feeding
controller and an existing-ball launch boundary. Then run actual settling and
single-ball contact tests before a 100-ball run at multiple head poses. Record
both successes and jams, rather than treating a commanded feeder index as a
confirmed launch.
