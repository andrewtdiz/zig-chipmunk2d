# Post-Review Remediation Plan

This document captures the open work items identified across the combined migration reviews. The objective is to bring the Zig port to full parity with Chipmunk2D in behavior, performance, and memory discipline.

## Task Checklist

- [ ] **Collision & Narrow-Phase Restoration**
    - [ ] Port the full Chipmunk GJK/EPA path plus `ClosestPointsNew`/contact clipping so segment–segment and segment–poly pairs emit multi-contact manifolds with correct normals.
    - [ ] Reintroduce cached collision IDs tied to the space allocator and ensure per-contact data feeds arbiters for persistence/warm-starting.
    - [ ] Move polygon scratch allocations off `std.heap.page_allocator` and into the layered allocator strategy owned by each `cpSpace`.
    - [ ] Reorder `postSolve` so callbacks execute after solver iterations with access to final impulses.

- [ ] **Spatial Indexing & cpSpace Lifecycle**
    - [ ] Finish porting `cpSpaceHash`, `cpSweep1D`, `space_step.zig`, and `space_query.zig` so the broad phase no longer double-loops over every shape pair and segment queries are exposed.
    - [ ] Extend `cpSpace` with stamps, sleeping components, cached arbiter maps, wildcard handler tables, and activation/wake-up paths per Chipmunk’s pipeline.
    - [ ] Split dynamic/static body partitions and maintain stamp counters so sleeping metadata can cull inactive islands.
    - [ ] Wire spatial indices into `cpSpace.step`/`cpHastySpace` so cached broad-phase data drives collision pair generation.

- [ ] **Constraint Solver & Arbiters**
    - [ ] Implement real `preStep`, `applyCachedImpulse`, and `applyImpulse` phases that compute effective mass, bias, warm-start impulses, and iterative solver updates for every constraint.
    - [ ] Add friction, bias, and restitution solving to the contact impulse loop so solver behavior matches Chipmunk’s.
    - [ ] Expand `cpArbiter` to store per-contact impulses, friction data, collision IDs, and persistent contact metadata.
    - [ ] Introduce pooled contact buffers and arbiter pools owned by the space allocator to reuse contacts across frames.

- [ ] **Memory & Allocator Discipline**
    - [ ] Replace global `AutoHashMapUnmanaged` collision ID caches with space-scoped structures that deinit explicitly.
    - [ ] Audit extras modules (`march.zig`, `space_debug.zig`, etc.) to ensure they allocate through the layered allocator design and release resources at teardown.
    - [ ] Provide allocator-backed storage for constraint-specific runtime data so solver coefficients persist without per-step heap churn.

- [ ] **Performance & Multithreading**
    - [ ] Update `resolveCollisions` to consume spatial index results rather than rechecking every shape pair each step.
    - [ ] Parallelize broad-phase collision generation inside `cpHastySpace` so collision detection scales with worker threads.
    - [ ] Cache and reuse arbiter/contact data between iterations to avoid recomputing manifolds every frame.

- [ ] **Verification & Testing**
    - [ ] Build the cross-implementation harness that runs Zig simulations alongside Chipmunk 7.0.3 for behavioral parity checks.
    - [ ] Add fuzz/property-based tests and long-running determinism loops to prove stability across seeds and time horizons.
    - [ ] Validate `cpHastySpace` performance and feature parity against the C implementation and document the results.

---

## Detailed Tasks

### 1. Collision & Narrow-Phase Restoration

*   **GJK/EPA & Contact Clipping**:
    *   Implement the Chipmunk GJK/EPA stack, `ClosestPointsNew`, and final clipping pipeline to restore full manifolds.
    *   Feed generated contacts into arbiters with persistent IDs and cached impulses.
*   **Allocator Discipline**:
    *   Replace polygon scratch allocations with per-space arenas or pools.
*   **Callback Ordering**:
    *   Ensure `postSolve` fires only after solver iterations complete and impulses are finalized.

### 2. Spatial Indexing & cpSpace Lifecycle

*   **Index Ports**:
    *   Complete `space_hash.zig`, `sweep1d.zig`, `space_step.zig`, and `space_query.zig`.
*   **cpSpace State**:
    *   Add stamp counters, sleeping components, cached arbiter maps, wildcard handlers, and dynamic/static partitions.
*   **Integration**:
    *   Drive `cpSpace.step` and `cpHastySpace` from the indices rather than brute-force loops.

### 3. Constraint Solver & Arbiters

*   **Constraint Phases**:
    *   Flesh out base hooks to mirror Chipmunk’s solver order (mass/bias setup, cached impulse application, iterative impulses).
*   **Arbiter Data**:
    *   Store per-contact impulse/ friction/ bias data and manage them through pooled buffers tied to the space allocator.
*   **Contact Persistence**:
    *   Keep contacts alive across frames with collision IDs and cached impulses for warm starting.

### 4. Memory & Allocator Discipline

*   **Space-Scoped Resources**:
    *   Tie collision ID maps, contact pools, and extras allocations to space allocators with explicit init/deinit.
*   **Constraint Storage**:
    *   Provide allocator-backed storage for constraint runtime coefficients to minimize per-step allocations.

### 5. Performance & Multithreading

*   **Broad-Phase Scalability**:
    *   Replace O(n²) loops with spatial index queries in both sequential and threaded steppers.
*   **Parallel Collision Detection**:
    *   Spread broad-phase pair generation across worker threads inside `cpHastySpace`.
*   **Solver Efficiency**:
    *   Reuse cached arbiters/contacts to reduce per-step recomputation.

### 6. Verification & Testing

*   **Parity Harness**:
    *   Compare Zig vs. Chipmunk 7.0.3 results across standard scenarios.
*   **Fuzzing & Determinism**:
    *   Add property tests and long deterministic runs; track any divergence.
*   **cpHastySpace Validation**:
    *   Measure multithreaded performance and correctness against the C baseline.

