# MiniEngine

MiniEngine is a small C++17 gameplay sandbox engine: a hand-rolled linear algebra library, a
transform-hierarchy scene graph over an entity/component store, a deterministic fixed-timestep
loop, rigid-body physics with broadphase/narrowphase/impulse resolution and raycast/sweep/overlap
queries, a data-driven immediate-mode UI layer with a headless focus model, and a frame profiler
with a headless benchmark harness. All simulation and UI logic lives in a window-free static
library (`minieng_core`) that builds and tests without a display; a GLFW + Dear ImGui application
is an optional front-end that renders the same core with a debug-line viewport and live tuning
widgets.

## Feature list → source files

| Feature | Files |
| --- | --- |
| Vector / quaternion / 4×4 matrix math, TRS compose-decompose, slerp, camera matrices | `engine/include/minieng/math.hpp`, `engine/src/math.cpp` |
| Scene graph (nodes, parents/children, cached world matrices, reparenting) | `engine/include/minieng/scene.hpp`, `engine/src/scene.cpp` |
| Entity/component world (dense arrays + id→index map, deterministic id order) | `engine/include/minieng/world.hpp`, `engine/src/world.cpp` |
| Deterministic xorshift64 RNG | `engine/include/minieng/rng.hpp`, `engine/src/rng.cpp` |
| Fixed-timestep loop with explicit system-stage ordering | `engine/include/minieng/loop.hpp`, `engine/src/loop.cpp` |
| Rigid-body physics: integration, SAP + brute-force broadphase, AABB/sphere/OBB narrowphase, impulse + friction resolution | `engine/include/minieng/physics.hpp`, `engine/src/physics.cpp` |
| Raycast / AABB sweep / overlap-sphere query API | same files (`PhysicsWorld`) |
| Data-driven UI layout (hand-written JSON parser, anchors, row/column stacking) | `engine/include/minieng/json.hpp`, `engine/src/json.cpp`, `engine/include/minieng/ui/layout.hpp`, `engine/src/ui/layout.cpp` |
| Binding table wiring UI names to live simulation state | `engine/include/minieng/ui/bindings.hpp`, `engine/src/ui/bindings.cpp` |
| Headless focus model (spatial + Tab traversal, activation, wrapping) | `engine/include/minieng/ui/navigation.hpp`, `engine/src/ui/navigation.cpp` |
| Deterministic replay records | `engine/include/minieng/replay.hpp`, `engine/src/replay.cpp` |
| Frame/system profiler and `MINIENG_PROFILE` | `engine/include/minieng/profiler.hpp`, `engine/src/profiler.cpp` |
| GLFW + Dear ImGui sandbox app (scene lines, inspector, sliders, replay controls) | `app/src/main.cpp` |
| Headless benchmark harness | `tools/bench.cpp` |
| Tests | `tests/test_math.cpp`, `tests/test_scene.cpp`, `tests/test_loop.cpp`, `tests/test_physics.cpp`, `tests/test_ui.cpp`, `tests/test_determinism.cpp`, `tests/test_query_fuzz.cpp` |
| UI layout data | `data/ui/main.ui.json`, `data/ui/alt.ui.json`, `docs/ui-layout.md` |
| Replay data | `data/replays/demo.replay` |
| Bench output | `results/bench.json`, `results/bench.txt` |

## Architecture

### Core / app split

`minieng_core` (static library, `engine/`) has **no** windowing, rendering, or third-party
dependency. It is the only thing CI builds and `ctest` exercises. The GLFW/ImGui app
(`app/`, gated behind `MINIENGINE_BUILD_APP`) and the benchmark (`tools/`) both link the same
core and are built from the same CMake project.

The core is window-free on purpose: every behaviour that matters (transform composition,
integration, collision response, layout math, focus traversal, replay determinism) is a pure
function of its inputs, so it can be tested headlessly and identically on every machine. If the
core needed a GL context, CI would need a display and the determinism claims would be
unverifiable.

### System update order

`FixedLoop` runs one fixed `dt = 1/60 s` step per tick and always executes stages in this order:

```
                 frame delta
                      |
                      v
              +----------------+       accumulator >= dt ?
              |  FixedLoop     |-------------------------------------+
              |  accumulator   |                                     |
              +----------------+                                     v
                                                       +--------------------------+
                                                       | 1. Input                 |
                                                       |    replay / devices      |
                                                       +--------------------------+
                                                                     |
                                                                     v
                                                       +--------------------------+
                                                       | 2. Spawn (script/logic)  |
                                                       +--------------------------+
                                                                     |
                                                                     v
                                                       +--------------------------+
                                                       | 3. PhysicsIntegrate      |  v += g*dt
                                                       |    semi-implicit Euler   |  x += v*dt
                                                       |    quaternion derivative |  q += 0.5*w*q*dt
                                                       +--------------------------+
                                                                     |
                                                                     v
                                                       +--------------------------+
                                                       | 4. CollisionResolve      |
                                                       |    broadphase -> narrow  |
                                                       |    -> positional+impulse |
                                                       +--------------------------+
                                                                     |
                                                                     v
                                                       +--------------------------+
                                                       | 5. SceneTransform        |
                                                       |    world = parent * local|
                                                       +--------------------------+
                                                                     |
                                                                     v
                                                       +--------------------------+
                                                       | 6. UiObserve             |
                                                       |    profiler, HUD, obs.   |
                                                       +--------------------------+
```

### Scene-graph transform composition

Nodes are stored flat (`SceneNode` pool) with parent indices. `update_world_transforms()` does a
single top-down pass: roots resolve first, then descendants, so one traversal leaves every cached
world matrix correct.

```
world(leaf) = world(mid) * local(leaf)
            = world(root) * local(mid) * local(leaf)
            = (T_root * R_root * S_root) * (T_mid * R_mid * S_mid) * (T_leaf * R_leaf * S_leaf)
```

`reparent(node, new_parent, keep_world=true)` captures the node's world matrix first, then solves
`local = inverse(world(new_parent)) * world(old)` and decomposes it back into TRS, so the node
stays put in world space.

### Broadphase → narrowphase → resolve

```
  entities with Collider + TransformRef
              |
              v
   world-space AABBs                      (OBB AABB = centre +/- sum|axis_i * h_i|)
              |
     +--------+---------+
     |                  |
  sweep-and-prune    brute force O(n^2)     <- selectable via config.use_sap
  (sort on X,       (every overlapping
   early-out)        pair, same test)
     |                  |
     +--------+---------+
              |
              v
   candidate pairs  (identical sets; verified by the fuzz/broadphase tests)
              |
              v
  narrowphase: AABB-AABB | Sphere-Sphere | OBB-OBB (SAT, 15 axes) | Sphere-OBB
              |           -> normal (a -> b) + penetration depth
              v
  resolve:  positional correction (Baumgarte * (penetration - slop) / (1/mA + 1/mB))
            normal impulse  j = -(1+e) * v_rel·n / (1/mA + 1/mB + angular terms)
            Coulomb friction clamped to mu * |j| on the tangent
              |
              v
   velocities + positions written back; SceneTransform refreshes world matrices
```

Static bodies have inverse mass/inertia 0, so they never move. Restitution is suppressed below
`restitution_slop` closing speed to stop resting jitter.

### UI layout → binding → focus

```
 data/ui/main.ui.json
        |  (hand-written JSON parser, minieng/json.hpp)
        v
   Layout::parse  ->  Widget tree (id, type, binding, min/max, anchor, direction)
        |  compute_layout(viewport_w, viewport_h)
        v
   computed Rects (anchors + row/column stacking; nothing hard-coded in pixels)
        |
        +--> UiContext::set_items_from_layout  -> focus items (id + binding + rect)
        |           move_focus(Right/Left/Up/Down)  spatial nearest, wraps
        |           move_focus(Next/Prev)           Tab order, wraps
        |           activate()                      fires the bound action
        |
        +--> BindingTable: "physics.gravity.y" -> getter/setter over PhysicsConfig
                        "selected.position.x" -> getter/setter over the selected entity
                        actions: sim.pause / sim.step / sim.reset
        |
        +--> ImGui renderer (app only): draws the same widget tree, positions every
             control at its computed rect, highlights the focused widget
```

Editing `data/ui/main.ui.json` changes the widget tree, the computed layout, and the focus order
with **no recompilation**. `tests/test_ui.cpp` asserts this explicitly by loading a second fixture
(`data/ui/alt.ui.json`) that adds a panel and checking that the tree grows.

## How to run

Requires CMake ≥ 3.16, a C++17 compiler, and network access the first time (CMake fetches
GoogleTest, GLFW and Dear ImGui via `FetchContent`).

```bash
# Headless core + tests (this is what CI runs)
cmake -S . -B build -DMINIENGINE_BUILD_APP=OFF
cmake --build build -j
ctest --test-dir build --output-on-failure

# Windowed sandbox app
cmake -S . -B build-app -DMINIENGINE_BUILD_APP=ON
cmake --build build-app -j
./build-app/app/minieng_sandbox                 # or --frames 300 to auto-exit

# Benchmark (headless)
cmake --build build --target minieng_bench -j
./build/tools/minieng_bench > results/bench.txt
```

Useful options: `-DMINIENGINE_BUILD_APP=OFF` (default ON), `-DMINIENGINE_BUILD_TESTS=ON`,
`-DMINIENGINE_BUILD_TOOLS=ON`, `-DMINIENGINE_WERROR=ON` (warnings as errors on first-party
targets only).

Sandbox controls: arrow keys / WASD / D-pad move focus, Tab / Shift+Tab cycle widgets,
Enter/Space activates, left-click in the viewport selects the entity under the cursor.

### Changing the UI without rebuilding

`data/ui/main.ui.json` is read at startup. Add a widget, change a binding, or rename a panel and
relaunch the app — no CMake run, no compile. The layout schema and the full list of binding names
are documented in [`docs/ui-layout.md`](docs/ui-layout.md). You can also point the app at another
file with `--ui path/to/layout.json`.

## Results

All numbers below were produced by this build on the machine listed, and are copied from
`results/bench.txt` / `results/bench.json` (benchmark) and from `ctest` (tests). Nothing here is
estimated or rounded up beyond the printed precision.

**Machine:** Apple M3, 8 cores, macOS 27.0.1, Apple clang 21.0.0, `CMAKE_BUILD_TYPE=RelWithDebInfo`.
Engine code is single-threaded; benchmark timings are wall-clock and therefore sensitive to
background load.

### Benchmark scenario (a) — raw table (verbatim from `results/bench.txt`)

```
=== (a) broadphase: brute force vs sweep-and-prune ===
  bodies  broadphase     mean_ms     p95_ms     max_ms        pairs     contacts
     500  bruteforce       0.484      0.667      8.526         64.8         64.8
     500  sap              0.125      0.173     15.068         64.8         64.8
    2000  bruteforce      12.894     30.793    111.639        307.5        307.5
    2000  sap              1.121      1.826     34.551        307.5        307.5
    5000  bruteforce      56.056    110.309    299.329       1008.2       1008.2
    5000  sap              2.951      3.967     21.823       1008.2       1008.2
```

1800 ticks per configuration, 60 warm-up ticks discarded, fixed seeds, zero gravity (bodies are
spread through a 60³ box so the measurement is broadphase/query cost rather than solver
convergence). "pairs" and "contacts" are per-tick averages.

### Frame time and broadphase speedup

| Bodies | Brute force mean / p95 (ms) | Sweep-and-prune mean / p95 (ms) | Speedup (mean) | Candidate pairs / tick |
| ---: | ---: | ---: | ---: | ---: |
| 500 | 0.484 / 0.667 | 0.125 / 0.173 | 3.9× | 64.8 |
| 2000 | 12.894 / 30.793 | 1.121 / 1.826 | 11.5× | 307.5 |
| 5000 | 56.056 / 110.309 | 2.951 / 3.967 | 19.0× | 1008.2 |

Both broadphases produce the **same** candidate pairs and the same contacts (1008.2 per tick at
5000 bodies); only the cost differs. That equality is asserted every test run by
`test_physics.cpp` (200 randomized scenes) and `test_query_fuzz.cpp`.

### Per-system frame breakdown — 5000 bodies, sweep-and-prune

| System | Mean (ms) | Share |
| --- | ---: | ---: |
| integrate | 0.093 | 3.2% |
| broadphase | 2.451 | 83.1% |
| narrowphase | 0.159 | 5.4% |
| resolve | 0.099 | 3.4% |
| transforms | 0.154 | 5.2% |
| **tick total** | **2.951** | **100.0%** |

The broadphase share includes recomputing 5000 world-space AABBs and sorting them each tick, which
is what dominates at this body count.

### Query throughput — 100k queries against the 5000-body scene

| Query | Count | Wall time (ms) | Throughput | Hit rate |
| --- | ---: | ---: | ---: | ---: |
| Raycast (sphere / AABB / OBB, nearest hit) | 100000 | 1936.991 | **51,626 /s** | 1.00 |
| AABB sweep (tunnelling-safe) | 100000 | 1699.678 | **58,835 /s** | 0.92 |

### Determinism

| Check | Result |
| --- | --- |
| Same seed, two independent runs (300 bodies, 300 ticks) | identical — `0x6bd6f1ddbf955f2f` |
| Different seed | differs — `0xd85a97ba80c6d4f5` |
| Replay of `data/replays/demo.replay` (1200 recorded ticks) | every per-tick hash matches |

The replay check is `tests/test_determinism.cpp`: it records 1200 ticks of input plus the resulting
world hash into `data/replays/demo.replay`, replays the file, and requires identical per-tick
hashes. It also verifies that perturbing a single input is detected.

### Tests

| Item | Value |
| --- | --- |
| `ctest` targets | 7 |
| GoogleTest cases | 66 |
| `ctest` wall time | 0.34 s |
| Failures | 0 |

### Lines of code

| Component | Lines |
| --- | ---: |
| `engine/` (window-free core) | 4429 |
| `tests/` | 1984 |
| `app/` (GLFW + Dear ImGui) | 730 |
| `tools/` (benchmark) | 436 |
| **Total** | **7579** across 34 `.hpp`/`.cpp` files |

### Caveats on these numbers

- Timings are wall-clock on a laptop with background load, so absolute `mean_ms` values move
  between runs even at fixed seeds. A previous run of the same binary measured the 5000-body SAP
  tick at 8.59 ms mean / 24.61 ms p95 (and 16.1k raycasts/s) under heavier load; the paired
  brute-force-vs-SAP comparison and the deterministic outputs (pair counts, hashes) are stable.
- `min`-of-`p50` numbers are unaffected by warm-up because 60 ticks are discarded.

## Limitations / next steps

- **No continuous collision detection for rotating OBBs.** `sweep_aabb` is exact for
  translationally-moved boxes (Minkowski expansion + slab test) but reduces every obstacle to its
  world AABB; a fast-rotating OBB can still tunnel.
- **Single contact point per pair.** Impulses are applied at one averaged contact point, and body
  inertia is a scalar approximation of the box/sphere diagonal, so resting stacks are stable but
  not perfectly physical.
- **No constraint solver** (no joints, motors, or warm-started sequential impulses across
  contacts); `solver_iterations` re-applies per-pair impulses rather than solving a global system.
- **Single-threaded.** No job system or SIMD; the broadphase is a serial sort-and-sweep.
- **Physics bodies are scene roots.** Integration writes local transforms, so a physics body that
  is also a child of a moving parent is not supported.
- **Fixed timestep only.** Spiral-of-death protection clamps the accumulator rather than
  interpolating; `FixedLoop::alpha()` exists for render interpolation but the line renderer does
  not use it.
- **The UI is ergonomic, not themed.** No docking, styling system, or text input; the ImGui
  renderer positions every control from the parsed layout rects.
- **Nothing here is pushed to a remote.** The repository is local-only.
