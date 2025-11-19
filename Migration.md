# Chipmunk2D → Zig Migration Plan

This document outlines a concrete plan to migrate the full `chipmunk2d` C implementation in `./chipmunk2d` into a native Zig library while preserving behavior, performance characteristics, and API surface. The end result is an idiomatic Zig library (no C ABI surface required) that maintains the same observable behavior for consumers unless a change is strictly beneficial to performance or safety.

The primary inputs for this plan are the upstream documentation in `chipmunk2d/Docs`:

- `1-Chipmunk2D Overview.md`
- `2-Core Physics Engine.md`
- `3-Collision Detection System.md`
- `4-Collision Shapes.md`
- `5-Core Types and Geometry Utilities.md`

and the C headers and sources under `chipmunk2d/include/chipmunk` and `chipmunk2d/src`.

---

## 1. Goals and Constraints

- **Functional coverage**
  - Port *all* functionality currently implemented in `chipmunk2d/src` and exposed via `chipmunk2d/include/chipmunk/*.h`.
  - Match simulation behavior (within floating point tolerance) to Chipmunk 7.0.3.
- **API compatibility (behavior, not ABI)**
  - Preserve the functional behavior and semantics of the public C API (as described in the upstream docs and headers).
  - Expose this behavior through an idiomatic Zig API (no requirement for a C ABI-compatible surface or drop-in C replacement).
- **Determinism and performance**
  - Preserve core algorithms and data structures (GJK/EPA, solver loop, spatial indices, contact persistence, sleeping, object pools).
  - Avoid regressions in asymptotic complexity; aim for similar or better runtime and memory behavior.
- **Incremental migration**
  - Allow stepwise replacement of C modules with Zig implementations while keeping tests green.
  - Keep a clear mapping from each `*.c`/`*.h` file to corresponding Zig modules.
- **Portability**
  - Continue to support all platforms currently supported by Zig plus any platform Chipmunk targets where feasible.

Non-goals:

- Re-designing the physics engine or changing the underlying algorithms.
- Adding major new features beyond what upstream Chipmunk offers (can be future work).

---

## 2. Target Zig Architecture

### 2.1 Package layout

Proposed structure under `src/`:

- `zig_chipmunk2d/`
  - `core/`
    - `types.zig` (cpFloat, cpBool, cpTimestamp, configuration, etc.)
    - `vect.zig` (cpVect and 2D math)
    - `bb.zig` (cpBB)
    - `transform.zig` (cpTransform)
  - `util/`
    - `pool.zig` (contact buffers, arbiter pools and other helpers that are not covered by Zig stdlib containers)
  - `shape/`
    - `shape_base.zig` (cpShape + vtable)
    - `circle.zig` (cpCircleShape)
    - `segment.zig` (cpSegmentShape)
    - `poly.zig` (cpPolyShape)
  - `collision/`
    - `collision.zig` (cpCollisionInfo, GJK/EPA)
    - `arbiter.zig` (cpArbiter)
  - `space/`
    - `space.zig` (cpSpace core, properties)
    - `space_step.zig` (cpSpaceStep pipeline)
    - `space_query.zig` (queries)
    - `sleeping.zig` (components, sleeping logic)
  - `spatial_index/`
    - `interface.zig` (cpSpatialIndex abstraction)
    - `bbtree.zig` (cpBBTree)
    - `space_hash.zig` (cpSpaceHash)
    - `sweep1d.zig` (cpSweep1D)
  - `constraint/`
    - `constraint_base.zig` (cpConstraint)
    - one module per constraint: pin, pivot, slide, gear, ratchet, rotary_limit, damped_spring, damped_rotary_spring, simple_motor, groove, etc.
  - `extras/`
    - `hasty_space.zig` (cpHastySpace / multithreaded stepping)
    - `march.zig` (cpMarch)
    - `polyline.zig` (cpPolyline)
    - `robust.zig` (cpRobust)
    - `space_debug.zig` (cpSpaceDebug)

`src/root.zig` exposes the public Zig API for the physics engine (no separate C-ABI layer).

### 2.2 Naming and style

- In the public Zig API, prefer idiomatic Zig naming (e.g. `Space`, `Body`, `step`, `init`) while keeping a clear mapping to the underlying Chipmunk concepts.
- Where a 1:1 mapping to a Chipmunk symbol is useful (for cross-referencing upstream docs), keep the original `cp*` name in an internal or compatibility module and re-export an idiomatic alias.
- Prefer `struct` methods and plain Zig functions over manual vtables where the C design uses function pointers, but keep behavior compatible (especially for `cpShapeClass` and `cpConstraintClass`).

---

## 3. Core Types and Geometry (Phase 1)

Based on `5-Core Types and Geometry Utilities.md` and headers like `chipmunk_types.h`, `cpVect.h`, `cpBB.h`:

### 3.1 Basic types and configuration (`core/types.zig`)

Port:

- `cpFloat`, `cpBool`, `cpHashValue`, `cpCollisionID`, `cpDataPointer`, `cpCollisionType`, `cpGroup`, `cpBitmask`, `cpTimestamp`.
- Compile-time configuration for `cpFloat` (single vs double precision) using Zig `comptime` constants instead of preprocessor macros.

Design:

- Use `pub const cpFloat = f64;` by default, with a `comptime` configuration to allow substitution (e.g. via `-DcpFloat=f32`).
- Represent bitmasks and IDs with fixed Zig integer types chosen for clarity and determinism (no strict FFI layout requirements, but keep types stable).

### 3.2 Vector math (`core/vect.zig`)

From `cpVect.h` and the docs:

- Implement constructors and operations (`cpv`, `cpveql`, `cpvadd`, `cpvsub`, `cpvmult`, `cpvlength`, `cpvdot`, `cpvperp`, `cpvnormalize`, `cpvlerp`, `cpvslerp`, etc.).
- Ensure all functions behave identically to the C implementation, including edge cases (zero-length vectors, clamping behavior).

Migration strategy:

- Port each function as a direct translation from C to Zig while writing tests that compare against the original C behavior (see Section 9).
- Use Zig `@floatCast`, `@intCast`, and `@fabs` equivalents to model the original `cpf*` utility functions.
- Where it simplifies and speeds up math, use the [`zmath` library](ZMATH_API.md) for SIMD vector operations (e.g. `F32x4`, `dot2`, `length2`, `normalize2`), while keeping the public `cpVect` API and semantics unchanged. Treat this as an internal implementation detail, not a user-visible type change.

### 3.3 Bounding boxes and transforms (`core/bb.zig`, `core/transform.zig`)

From `cpBB.h` and `cpTransform` definitions:

- Implement `cpBB` struct and operations (constructors, intersection/union, containment, expansion).
- Implement `cpTransform` as a 2×3 affine transform with helpers like `cpTransformTranslate`, `cpTransformRotate`, and methods to transform vectors and bounding boxes.
- For more complex linear algebra (e.g. composing transforms, interpolation, or future 3D/debug utilities), prefer reusing `zmath` matrix/quaternion primitives (`Mat`, `Quat`, rotation/translation helpers) where they fit naturally, again keeping the public 2D `cpTransform` behavior identical to C.

---

## 4. Utility Containers and Infrastructure (Phase 2)

Based on `2-Core Physics Engine.md` and source files like `cpArray.c`, `cpHashSet.c`, `cpSpatialIndex.c`:

### 4.1 Object pools and contact buffers (`util/pool.zig`)

From `2-Core Physics Engine.md` and `cpSpace.c` / `cpSpaceStep.c`:

- Implement contact buffer ring management (`contactBuffersHead`, `allocatedBuffers`) and arbiter pooling.
- Keep the semantics of `collisionPersistence` and contact recycling identical to C.

Design:

- Represent pools using `std.ArrayList` and/or plain slices plus free lists; rely on the caller to pass an `Allocator`.
- Use Zig standard library collections (`std.ArrayList`, `std.HashMap`/`std.AutoHashMap`) wherever they provide equivalent functionality to Chipmunk’s custom containers.
- Provide explicit `init` / `deinit` operations and keep lifetime management centralized in `space/space.zig`.

---

## 5. Collision Shapes (Phase 3)

Guided by `4-Collision Shapes.md`, `cpShape.h`, `cpPolyShape.h`, `cpShape.c`, `cpPolyShape.c`:

### 5.1 Shape base class (`shape/shape_base.zig`)

- Port `cpShape`, `cpShapeClass`, and `cpShapeFilter`.
- Represent the virtual method table (`cacheData`, `pointQuery`, `segmentQuery`, `destroy`) using a Zig `struct` of function pointers or an `enum` + `switch` where appropriate.
- Implement common properties: sensor flag, elasticity, friction, surface velocity, filter, collisionType, userData, `cpShapeMassInfo`.

### 5.2 Concrete shapes (`shape/circle.zig`, `shape/segment.zig`, `shape/poly.zig`)

- Implement `cpCircleShape`, `cpSegmentShape`, `cpPolyShape` structs and their constructors, property accessors, and cache/update functions.
- Port neighbor support for segments (`cpSegmentShapeSetNeighbors`) and inline allocation optimization for `cpPolyShape` (`CP_POLY_SHAPE_INLINE_ALLOC`).
- Ensure bounding box and transformed-geometry calculations match the C implementation.

### 5.3 Shape queries and filtering

- Implement `cpShapePointQuery`, `cpShapeSegmentQuery`, `cpShapeCacheBB`, etc., delegating to type-specific implementations via the vtable.
- Maintain collision filtering semantics using `cpShapeFilter`, `cpGroup`, and `cpBitmask`.

---

## 6. Collision Detection System (Phase 4)

Based on `3-Collision Detection System.md` and `cpCollision.c`:

### 6.1 Narrow-phase dispatch (`collision/collision.zig`)

- Implement `cpCollide` dispatcher that selects appropriate collision functions based on shape types:
  - `CircleToCircle`, `CircleToSegment`, `SegmentToSegment`, `CircleToPoly`, `SegmentToPoly`, `PolyToPoly`.
- Port `cpCollisionInfo` and related contact representations.

### 6.2 GJK and EPA

- **Status**: Not yet ported. The current Zig narrow-phase uses SAT-style polygon tests and closest-point helpers for the covered shape pairs; GJK/EPA will be brought over in a later Phase 3 milestone to regain full parity with Chipmunk's penetration resolution path.
- Translate the GJK implementation to Zig:
  - Support for cached collision IDs (`cpCollisionID`), Minkowski support points, recursion/iteration structure.
  - Early exit conditions and tolerance matching the C code.
- Translate EPA for penetration depth and separating axis computation:
  - Hull management, closest-edge search, duplicate support point detection.
- Preserve algorithm invariants to ensure stable behavior and performance.

### 6.3 Contact generation and clipping

- Port `ClosestPointsNew`, `ContactPoints`, and related helpers that convert GJK/EPA results into actual contact points used by `cpArbiter`.
- Ensure that the number and ordering of contacts matches the original C logic as closely as possible (important for determinism).

---

## 7. Physics World and Simulation (Phase 5)

Based on `2-Core Physics Engine.md` and the implementation in `cpSpace.c`, `cpSpaceStep.c`, `cpSpaceQuery.c`, `cpArbiter.c`:

### 7.1 Space data structures (`space/space.zig`)

- Implement `cpSpace` structure with:
  - `dynamicBodies`, `staticBodies`, `sleepingComponents` (arrays of bodies/components).
  - `staticShapes`, `dynamicShapes` (spatial indices).
  - `constraints`, `arbiters`, `cachedArbiters`.
  - Contact buffers, collision handlers, post-step callbacks.
  - Physics properties (gravity, damping, iterations, collision slop/bias, persistence, sleep thresholds).
- Implement allocation and teardown patterns analogous to `cpSpaceAlloc`, `cpSpaceInit`, `cpSpaceNew`, `cpSpaceFree`.

### 7.2 Simulation pipeline (`space/space_step.zig`)

Port the `cpSpaceStep` pipeline as described in the docs:

1. Increment `space->stamp`.
2. Integrate positions (`position_func`).
3. Broad-phase collision update and `cpSpaceCollideShapes`.
4. Process connected components, sleeping logic, and contact graph.
5. Filter stale arbiters and handle separation callbacks.
6. Pre-step for arbiters and constraints.
7. Integrate velocities (`velocity_func`).
8. Apply cached impulses (warm starting).
9. Iterative impulse solver loop over arbiters and constraints.
10. Post-solve callbacks and post-step callbacks.

Design:

- Model the pipeline as a sequence of private helper functions with a public `cpSpaceStep(space: *cpSpace, dt: cpFloat) void`.
- Preserve locking semantics (`cpSpaceLock`/`cpSpaceUnlock`) using a `locked` counter and deferred operations list.

### 7.3 Queries and callbacks (`space/space_query.zig`)

- Port `cpSpacePointQuery`, `cpSpaceSegmentQuery`, `cpSpaceBBQuery`, `cpSpaceShapeQuery`.
- Implement locking and callback invocation semantics exactly as in C (lock space, query both spatial indices, call user callbacks, unlock).
- Implement collision handler registration and wildcard handlers as described in the docs.

### 7.4 Arbiters and contact management (`collision/arbiter.zig`)

- Implement `cpArbiter` lifecycle: creation, update, pre-step, cached impulse application, impulse solving, post-solve.
- Port contact buffer and arbiter caching logic using the pools created earlier.

---

## 8. Constraints and Joints (Phase 6)

From `cpConstraint.h` and the corresponding `cp*.c` files:

### 8.1 Constraint base (`constraint/constraint_base.zig`)

- Implement `cpConstraint` struct and class-like behavior (`cpConstraintClass`).
- Support callback hooks (`preSolve`, `postSolve`) and user data fields.
- Manage references to the two connected bodies and their local anchors/frames.

### 8.2 Individual constraint types

Port each constraint module one by one:

- `cpPinJoint`, `cpSlideJoint`, `cpPivotJoint`, `cpGrooveJoint`
- `cpDampedSpring`, `cpDampedRotarySpring`
- `cpGearJoint`, `cpRatchetJoint`, `cpRotaryLimitJoint`
- `cpSimpleMotor`

For each:

- Implement parameter storage, pre-step calculations, and impulse application code based on the C implementation.
- Add focused tests that validate known behaviors (e.g. distance maintenance for pin joints, gear ratios, limits).

---

## 9. Spatial Indexing and Optimization (Phase 7)

Based on `2-Core Physics Engine.md`, `cpSpatialIndex.h`, `cpBBTree.c`, `cpSpaceHash.c`, `cpSweep1D.c`:

### 9.1 Spatial index interface (`spatial_index/interface.zig`)

- Implement a Zig version of the `cpSpatialIndex` abstraction with function pointers or a tagged union.
- Provide a uniform API for insertion, removal, reindexing, and querying.

### 9.2 Concrete indices

- Port:
  - `cpBBTree` (bounding box tree).
  - `cpSpaceHash` (hash grid).
  - `cpSweep1D` (1D sweep-and-prune).
- Ensure that iterator order and performance characteristics remain close to the C implementation.

---

## 10. Extras and Advanced Features (Phase 8)

After the core engine is validated, port the remaining modules. These are still within the overall migration scope (the goal is to cover all upstream functionality), but they can be scheduled after the main engine, constraints, and spatial indices:

- `cpHastySpace` (multithreaded stepping) – consider mapping to Zig `std.Thread` and structured concurrency.
- `cpMarch` – marching squares implementation.
- `cpPolyline`, `cpRobust`, `cpSpaceDebug` – utility functions for debug rendering and polygon operations.

These modules depend on the core Zig port but are not required for the minimal simulation path and demos; they should still be ported before considering the migration “complete”.

---

## 11. Testing and Validation Strategy

To ensure behavior parity with the C implementation:

- **Reference tests against C**
  - For an initial phase, build the original C library as a separate module and call it from Zig tests using `@cImport`.
  - For each subsystem (vectors, shapes, collision, space stepping, constraints), write tests that:
    - Run the C version and the Zig version on identical inputs.
    - Compare outputs (positions, velocities, contact points, impulses) within a configurable tolerance.
- **Property and fuzz testing**
  - Use Zig’s fuzz testing (`std.testing.fuzz`) to generate random bodies, shapes, and constraints, and ensure no panics, invalid states, or assertion failures occur.
- **Determinism checks**
  - Run long simulations with fixed seeds and verify that results are stable across runs and between C and Zig implementations.
- **Performance benchmarks**
  - Compare stepping times between C and Zig implementations on representative scenes.

---

## 12. Migration Workflow and Milestones

Suggested order of implementation:

1. **Core math and types**
   - `core/types.zig`, `core/vect.zig`, `core/bb.zig`, `core/transform.zig`.
2. **Utility infrastructure and pools**
   - `util/pool.zig` and establishing patterns for using `std.ArrayList`, `std.HashMap`/`std.AutoHashMap`, and other Zig stdlib containers to replace Chipmunk’s custom arrays and hash sets.
3. **Shapes**
   - `shape/shape_base.zig`, then circle → segment → poly.
4. **Collision detection**
   - Narrow-phase dispatch, GJK/EPA, contact generation, arbiters.
5. **Space and simulation**
   - `space/space.zig`, `space/space_step.zig`, `space/space_query.zig`, locking, sleeping.
6. **Constraints**
   - Base constraint, then each concrete joint type.
7. **Spatial indices**
   - Interface, BBTree, SpaceHash, Sweep1D.
8. **Extras**
   - HastySpace, March, Polyline, Robust, SpaceDebug.
9. **Public Zig API and documentation**
   - Finalize the outward-facing Zig modules and document how they map back to Chipmunk2D concepts and original C APIs.

At each milestone, ensure:

- The Zig implementation for that subsystem passes parity tests against the C implementation.
- Public APIs needed by higher layers are stable and documented.

This plan should provide a clear path to a full Zig-native port of Chipmunk2D while allowing incremental progress and continuous validation against the original C library.
