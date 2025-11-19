# Feedback Remediation Task List

Derived from `FeedbackSummary.md`, `Feedback/Feedback_*.md`, `TaskList.md`, `Migration.md`, and `chipmunk2d/Docs/*`. This document enumerates the outstanding work needed to bring the Zig port into parity with Chipmunk2D.

## Task Checklist

- [x] **Collision Detection Completion**
    - [x] Port GJK/EPA narrow-phase (chipmunk2d `cpCollision.c`) with cached collision IDs.
    - [x] Implement `ClosestPointsNew`, `ContactPoints`, and multi-contact clipping so arbiters receive full penetration data.
    - [x] Fix segment–segment and segment–poly dispatch to use the proper narrow-phase routines (no `bbOverlap` fallback).

- [x] **Spatial Indexing & cpSpace Workflow**
    - [x] Replace the placeholder `cpSpatialIndex`/`cpBBTree` with real bounding-box tree logic (fat-leaf heuristics, rebalancing).
    - [x] Port `cpSpaceHash` and `cpSweep1D` modules and integrate them as optional indices.
    - [x] Integrate spatial indices into `cpSpace.step` (reindex, broad-phase queries) instead of double-looping all shapes.
    - [x] Reintroduce `space_step.zig` and `space_query.zig` modules to mirror the C pipeline and expose point/segment/bb/shape queries.

- [ ] **cpSpace Structure & Sleeping**
    - [ ] Split bodies and shapes into dynamic/static collections with sleeping components and stamp tracking.
    - [ ] Maintain cached arbiter maps keyed by shape pairs; add activation logic to wake sleeping islands.
    - [ ] Ensure `postSolve` callbacks run after the solver iterations, matching Chipmunk’s ordering.

- [ ] **Arbiters, Constraints, and Memory Pools**
    - [ ] Implement contact buffer pools / arbiter pools (`util/pool.zig`) tied to the space allocator.
    - [ ] Enhance `cpArbiter` to store cached impulses, friction, and bias data; expose warm-start hooks.
    - [ ] Flesh out `cpConstraint` base methods (`preStep`, `applyCachedImpulse`, `applyImpulse`) and wire them into joints/constraints.
    - [ ] Remove per-collision arena allocations; rely on space-level allocators for temporary data.

- [ ] **Advanced Modules & Multithreading**
    - [ ] Implement a true multithreaded `cpHastySpace` using `std.Thread` and work queues.
    - [ ] Add the missing `space_hash.zig` / `sweep1d.zig` source files referenced in the migration plan.

- [ ] **Verification & Testing Enhancements**
    - [ ] Create cross-implementation tests that call the original C library via `@cImport` for collisions, solver steps, and constraints.
    - [ ] Add fuzz/property tests (randomized shapes/bodies) and determinism checks across long simulations.

## Detailed Tasks

### 1. Collision Detection Completion
- Reuse Chipmunk’s GJK/EPA structure:
    - Minkowski support points, cached IDs, recursive simplex handling.
    - EPA hull expansion and duplicate-filtering.
- Implement the contact conversion helpers (`ClosestPointsNew`, `ContactPoints`) to turn GJK/EPA output into deterministic contact sets.
- Replace temporary arena allocations with reusable buffers (allocator passed from `cpSpace`).

### 2. Spatial Indexing & cpSpace Workflow
- Rebuild `cpSpatialIndex` with node structures, bounding-box expansion, and query callbacks equivalent to `cpBBTree.c`.
- Create `spatial_index/space_hash.zig` and `spatial_index/sweep1d.zig` based on Chipmunk’s C code.
- Update `cpSpace` to:
    - Maintain `staticShapes` and `dynamicShapes` indices.
    - Call `reindexStatic`, `reindexShapes`, and `reindexQuery` before narrow-phase collision.
    - Lock/unlock around queries and modifications, enforcing the same semantics as `cpSpaceLock`.
- Reintroduce `space_step.zig` for the main step pipeline (stamps, sleeping, component processing) and `space_query.zig` for query APIs including segment/raycast queries.

### 3. cpSpace Structure & Sleeping
- Add structures for sleeping components (`cpSpaceComponent` equivalent) and integrate them with body activation/deactivation.
- Track `stamp`, `sleepTimeThreshold`, `collisionBias`, and other configuration parameters.
- Implement handler lookup tables keyed by `cpCollisionType` with wildcard support (matching `cpSpace.c`).

### 4. Arbiters, Constraints, and Memory Pools
- Build a pooled contact buffer ring (Chipmunk’s `cpContactBufferHeader`) using Zig arenas/allocators.
- Have `cpArbiter` store per-contact impulses, tangential impulses, bias, and friction/elasticity values, along with `cpCollisionID`.
- Expand constraints:
    - Each joint implements `preStep` to compute mass/bias, `applyCachedImpulse` for warm starting, and `applyImpulse` for solver iterations.
    - Provide `ConstraintOps` per constraint hooking into `cpSpace`’s solver loop.
- Audit all temporary allocations (e.g., polygon collision vertex transforms) and route them through a per-space scratch allocator.

### 5. Advanced Modules & Multithreading
- Extend `cpHastySpace` with worker threads, job queues, and parallel sections for broad phase, constraint solving, or contact processing as in Chipmunk’s C implementation.
- Ensure the extras module exposes configuration knobs (thread count, chunk sizes) similar to the original API.

### 6. Verification & Testing Enhancements
- Build a test harness that:
    - Imports `chipmunk2d` C functions via `@cImport`.
    - Runs identical scenarios in C and Zig, comparing positions, velocities, contact points, and impulses.
- Add fuzzers/determinism tests:
    - Random body/shape creation, stepping, and invariants (no NaNs, consistent energy).
    - Long-running deterministic scenes to detect divergence.
- Expand existing unit/integration tests to cover segment queries, sleeping, handler callbacks, allocator pools, and multithreaded stepping.

---

Completing the above tasks will close the gaps highlighted in the migration feedback and bring the Zig implementation in line with the goals defined in `TaskList.md`, `Migration.md`, and the Chipmunk2D documentation.
