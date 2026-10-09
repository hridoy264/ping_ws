# Installed-engine continuation — 9 October 2026

These are labelled diagnostic copies of the immutable October8 model and ball load. They retain100 physical sphere entities, their original mass/radius/contact properties, the full377 robot collision pieces,1ms timestep and zero initial motor commands. `prepare_variants.py` records source and output hashes. The resource path is the frozen October8 mesh directory. No ball repositioning, runtime spawning, expiration or artificial launch is added.

On resume Docker Desktop was stopped. `docker desktop stop --force` confirmed it was not running, `docker desktop start` recovered it, and only `pingpong-r10-dev` was started. No previous trial processes remained.

The October8 Bullet collision detector with PGS had completed5.01 simulated seconds but failed:6.11mm maximum sampled pair penetration,24 final containment failures and one ball on the floor. This is not a capacity or feeding pass.

New diagnostics first isolated the original FCL/Dantzig slowdown. Both the17-collision hopper-only case and the377-collision static-robot case reached only0.801 simulated seconds in90 wall seconds. Removing articulated robot movement and most robot contacts did not remove the packed-pile slowdown.

The installed DART6.12.1 FCL detector defaults to tessellating spheres and boxes into meshes, rather than analytic primitive contacts. That is a plausible source of the dense contact problem, and motivates testing ODE and DART collision detection without altering mechanism geometry. The DART detector's unsupported-shape warnings, if any, must invalidate full-mechanism use. The native Bullet physics plugin is separately tested for compatibility; its presence does not establish support.

Sources inspected: installed `/usr/include/dart/constraint/PgsBoxedLcpSolver.hpp` and `/usr/share/sdformat12/1.8/physics.sdf`; upstream [DART6.12.1 FCL detector](https://github.com/dartsim/dart/blob/v6.12.1/dart/collision/fcl/FCLCollisionDetector.cpp) and [Gazebo Physics5 world configuration](https://github.com/gazebosim/gz-physics/blob/ign-physics5/dartsim/src/WorldFeatures.cc).

Run reports and `benchmark_summary.json` are the authority for completed native results. Generated input files alone are not runtime evidence.
