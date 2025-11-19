# Combined Migration Review Findings

This document consolidates all unique, high-priority review findings gathered from `FeedbackSummary.md` and `Review/review1.md` – `review4.md`.

## Collision & Narrow-Phase Coverage
- The narrow-phase path still lacks Chipmunk’s GJK/EPA implementation, cached collision IDs, and the `ClosestPointsNew`/contact clipping stages, so segment–segment and segment–poly pairs degrade to bounding-box overlap tests and never generate multiple contacts or correct normals.
- Polygon collisions allocate scratch data straight from `std.heap.page_allocator` instead of a space-owned allocator, bypassing the layered allocator plan.
- Collision handlers currently raise `postSolve` inside broad-phase pair creation, before any solver iterations, reversing Chipmunk’s callback ordering and denying callbacks access to the final impulse data.

## Spatial Indexing & cpSpace Pipeline
- `cpSpatialIndex` is just a linear array scan; `cpSpaceHash`, `cpSweep1D`, `space_step.zig`, and `space_query.zig` have not been ported, leaving O(n²) broad-phase traversal and no segment query support.
- `cpSpace` maintains only flat `ArrayList`s with a single collision handler and no stamps, sleeping components, cached arbiter maps, wildcard handler tables, or activation/wake-up paths, so island sleeping, handler lookup caching, and arbiter persistence are all missing.
- Dynamic/static body partitions and stamp counters referenced in the migration plan remain unimplemented, which prevents island culling and sleeping metadata from short-circuiting inactive bodies.
- Task list entries for sleeping, cached arbiters, handler tables, and spatial indices remain unchecked, confirming the lifecycle work is incomplete.

## Constraint Solver & Arbiter Fidelity
- Base constraint lifecycle hooks (`preStep`, `applyCachedImpulse`, `applyImpulse`) are empty, so joints never compute effective mass, bias, warm-start impulses, or iterative solver updates; constraint phases also run out of order relative to Chipmunk’s documented pipeline.
- The solver applies only basic normal impulses without friction or bias corrections, and `postSolve` fires before impulses are computed, so callbacks and frictional behavior diverge from the C engine.
- Arbiters are recreated every frame, only store up to four contacts, and omit cached normal/tangent impulses, friction/bias data, collision IDs, and pool ownership; consequently, contacts cannot warm-start, friction is ignored, and deterministic contact persistence is lost.
- Contact buffers and arbiter pools promised in the migration plan never materialized; all contact data is reallocated every step from `std.ArrayList`, increasing allocator churn and preventing reuse.
- Constraint implementations lack allocator-backed storage for per-constraint state, so no joint can retain solver coefficients between frames.

## Memory & Allocator Discipline
- Collision ID caching uses a global `AutoHashMapUnmanaged` backed by `std.heap.page_allocator` with no teardown hook, so IDs leak across simulations and ignore the per-space allocator strategy.
- Extras modules such as `march.zig` and `space_debug.zig` also allocate directly from the page allocator without releasing or routing through space-level allocators, undermining the layered allocator architecture.

## Performance & Multithreading
- `resolveCollisions` still double-loops over every shape pair, and `cpHastySpace` reuses that path verbatim, so both sequential and threaded stepping retain O(n²) collision costs despite the presence of spatial index stubs.
- `cpHastySpace` only parallelizes the constraint and arbiter phases; collision detection and spatial indexing remain single-threaded and unaccelerated, limiting scalability.
- Because arbiters and contacts are rebuilt from scratch, collisions are re-resolved each iteration with no cached impulses, increasing per-step work and erasing Chipmunk’s solver warm-start advantages.

## Verification & Remaining Work
- No cross-implementation test harness, fuzz/property suite, determinism loop, or `cpHastySpace` parity validation has been built, so behavior and performance regressions against Chipmunk 7.0.3 remain unchecked.
- Several deliverables marked complete in `TaskList.md` (sleeping, spatial indices, allocator pools, multithreaded parity) still diverge from the migration plan, so the project cannot yet claim migration parity.

