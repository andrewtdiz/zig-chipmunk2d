# Migration Feedback Summary

This document consolidates the findings across `Feedback/Feedback_1.md` through `Feedback/Feedback_5.md` (Feedback_4.md contains no additional content). The recurring themes are outlined below so the remaining migration work can be prioritized.

## 1. Collision Detection
- The narrow-phase path still lacks the Chipmunk GJK/EPA implementation, cached collision IDs, and the `ClosestPointsNew`/contact clipping pipeline. `src/collision/collision.zig` relies on SAT-style helpers plus a BB fallback, so segment–segment and segment–poly interactions devolve to bounding-box overlap tests and lose multiple contacts/normals.
- Without full contact generation, arbiters never receive the penetration data the C engine provides, which affects solver warm-starting, friction, and determinism.

## 2. Spatial Indexing & cpSpace Pipeline
- `cpSpatialIndex` is a linear array scan and `cpBBTree` simply wraps it; `cpSpaceHash`/`cpSweep1D` were never ported. As a result, `cpSpace.step` double-loops over every shape pair, giving O(n²) collision detection instead of the documented broad-phase acceleration.
- The `cpSpace` struct keeps only flat `ArrayList`s, omitting dynamic/static body partitions, sleeping components, stamp counters, cached arbiters, and per-shape spatial indices. There is no `space_step.zig` or `space_query.zig`, and segment queries are entirely missing.
- `postSolve` callbacks now run after the solver loop, but the rest of the broad-phase/sleeping pipeline still diverges from Chipmunk’s structure.

## 3. Arbiters, Constraints, and Memory Management
- Arbiters now use pooled contact buffers and retain cached impulses so contact reuse and warm-starting match Chipmunk’s behavior.
- Constraint base methods and joint implementations participate in the solver phases (`preStep`, `applyCachedImpulse`, `applyImpulse`) instead of only performing post-step corrections.
- Collision helpers allocate through the space allocator with pooled buffers instead of per-collision arenas, aligning with the layered allocator strategy.

## 4. Advanced Modules & Testing
- `cpHastySpace` merely calls the serial `cpSpace.step` and leaves the multithreaded solver unimplemented.
- The test strategy (cross-checking via `@cImport`, fuzz/property tests, determinism checks) has not been executed; existing tests are limited to small `std.testing` units, so behavior parity with Chipmunk 7.0.3 remains unverified.

## 5. Unfinished Milestones
- Several deliverables marked complete in `TaskList.md` do not exist in the tree: there are no `space_step.zig`, `space_query.zig`, `space_hash.zig`, or `sweep1d.zig` modules, and spatial indices are unused in `cpSpace`.
- Many of the above gaps (collision, sleeping, spatial indices, allocator pools, multithreaded stepping) are explicitly listed as remaining work in the TaskList/Migration plan, so they need to be implemented before claiming migration parity.

Addressing these items will align the Zig port with the documented Chipmunk2D behavior, restore the expected performance characteristics, and satisfy the migration plan’s requirements.
