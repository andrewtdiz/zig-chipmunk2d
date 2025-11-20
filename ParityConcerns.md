# Zig Chipmunk2D Port Parity Concerns

- Mass/area helpers diverge from the C reference:
  - `src/space/body.zig:105-107` computes segment inertia as `length_sq/12 + r^2/2`, dropping the center-offset term and box approximation used in `chipmunk2d/src/chipmunk.c:76-83`, so rounded segments get under-estimated inertia.
  - `src/shape/shape_base.zig:56-77` omits `cpfabs` for circle annulus area and removes the radius/perimeter term from polygon area that `chipmunk2d/src/chipmunk.c:70-127` uses; polygon area also ignores the radius parameter entirely, changing mass/density calculations.
  - `src/shape/shape_base.zig:99-119` adds a `mass * radius^2` term and lacks the 2-vertex fallback present in `chipmunk2d/src/chipmunk.c:92-111`, so polygon inertia and thin-segment handling no longer match.
  - `src/space/body.zig:114-116` changes the `cpMomentForBox2` API to require precomputed width/height/offset instead of accepting a `cpBB` like the C API (see `chipmunk2d/src/chipmunk.c` right after the poly helpers), breaking 1:1 API parity described in the docs.

- Shape feature gaps relative to C structs:
  - `src/shape/shape_base.zig:27-37` has no mass info, density, or cached area/moment fields (C keeps `cpShapeMassInfo` on cpShape), so APIs like `cpShapeSetMass/SetDensity` and `cacheData` hooks are missing.
  - `src/shape/poly.zig` stores only raw vertices and radius; it lacks splitting planes, bevel data, and convex-hull helpers (`cpLoopIndexes`, QuickHull, `cpConvexHull` from `chipmunk2d/src/chipmunk.c:158-260`), so polygon validation/hull building and beveled edge support are absent. `src/shape/segment.zig` similarly omits neighbor tangents (`cpSegmentShapeSetNeighbors`) used for smooth joins.

- Simulation pipeline ordering differs:
  - Chipmunk’s documented `cpSpaceStep` integrates positions, runs broad/narrow phase, processes components, pre-steps constraints, integrates velocities, iterates the solver, then fires post-solve callbacks.
  - The Zig stepper updates velocities before broad-phase work, skips component processing, and defers position integration until after post-solve/separation, which can alter contact persistence, sleep timing, and constraint biasing.

- Sleeping and component processing are simplified:
  - Chipmunk sleeps/wakes whole islands via `cpSpaceProcessComponents`; the Zig port only evaluates per-body kinetic energy (`src/space/space_step.zig:156-171`), so constraint-linked bodies can diverge.
  - Bodies do not track arbiter/shape/constraint lists like `cpBody` in C, and the stepper omits `cpSpaceFilterArbiters`-style safe removal, so callback ordering/removal semantics differ.

- Arbiter caching and allocations:
  - Collision/arbiter caching uses hash maps and fresh array appends each step (`src/space/space_step.zig:98-111`, `src/space/space_step.zig:199-219`) instead of the contact-buffer ring and pooled arbiters in C, increasing per-frame allocation and cache churn risk.

- Multithreaded broad-phase throughput:
  - `src/extras/hasty_space.zig` parallelizes `resolveCollisionsRange`, but each query path grabs a mutex around collision resolution, arbiter lookup, and handler callbacks; that shared lock serializes narrow-phase/arbiter append work, limiting the throughput gains Chipmunk’s hasty space design aims for.

- Missing diagnostics/version surface:
  - There is no `cpMessage`/version string equivalent from `chipmunk2d/src/chipmunk.c:33-58`, so documented error reporting and version queries are unavailable in the Zig port.
