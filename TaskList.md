# Chipmunk2D to Zig Migration Plan

This document outlines the strategy for migrating the Chipmunk2D physics engine from C to Zig. The goal is to rewrite the full Chipmunk2D functionality in idiomatic Zig while maintaining the performance, stability, and behavior of the original library. The end result is a Zig library (no C ABI surface required) whose API maps cleanly onto the original Chipmunk concepts.

## Task Checklist

- [x] **Project Setup & Infrastructure**
    - [x] Initialize Zig project (`zig init-lib`) (already done for this repo; kept here for reference)
    - [x] Configure `build.zig`
    - [x] Set up demo/benchmark executable
    - [x] Define memory management strategy (Allocator interface)

- [x] **Phase 1: Foundation (Math & Core Types)**
    - [x] Port Configuration and Core Types (`src/core/types.zig`: `cpFloat`, `cpBool`, collision types, IDs, bitmasks)
    - [x] Port Vector Math (`src/core/vect.zig`: `cpVect` operations)
    - [x] Port Bounding Boxes (`src/core/bb.zig`: `cpBB` operations)
    - [x] Port Transforms (`src/core/transform.zig`: `cpTransform`, any matrix helpers)

- [ ] **Phase 2: Physics Primitives**
    - [x] Port Rigid Bodies (`src/space/body.zig`: `cpBody` struct & logic)
    - [x] Define Shape Interface/Base (`src/shape/shape_base.zig`)
    - [x] Implement `CircleShape` (`src/shape/circle.zig`)
    - [x] Implement `SegmentShape` (`src/shape/segment.zig`)
    - [x] Implement `PolyShape` (`src/shape/poly.zig`)
    - [x] Define Constraint Interface/Base (`src/constraint/constraint_base.zig`)

- [ ] **Phase 3: Spatial Indexing & Collision Detection**
    - [x] Define Spatial Index Interface (`src/spatial_index/interface.zig`)
    - [x] Port `cpBBTree` (Spatial Index) (`src/spatial_index/bbtree.zig`)
    - [x] (Optional, later in migration) Port `cpSpaceHash` and `cpSweep1D` (`src/spatial_index/space_hash.zig`, `src/spatial_index/sweep1d.zig`)
    - [ ] Port GJK & EPA Algorithms (`src/collision/collision.zig`) — still pending; current narrow-phase uses SAT/closest-point helpers only
    - [x] Implement Primitive Collision Tests
    - [x] Implement `cpCollide` Dispatch
    - [x] Port Arbiters (`src/collision/arbiter.zig` & contact persistence)

- [x] **Phase 4: Constraints & Joints**
    - [x] Implement Simple Joints (`Pin`, `Slide`, `Pivot`)
    - [x] Implement Complex Joints (`Groove`, `DampedSpring`, `RotarySpring`)
    - [x] Implement Motors (`SimpleMotor`, `Gear`, `Ratchet`, `RotaryLimit`)

- [x] **Phase 5: The Space (Simulation Core)**
    - [x] Port `cpSpace` Struct (`src/space/space.zig`)
    - [x] Implement Object Management (`add`/`remove`)
    - [x] Implement Simulation Loop (`cpSpaceStep`/`step` in `src/space/space_step.zig`)
    - [x] Port Impulse Solver
    - [x] Implement Collision Handlers/Callbacks
    - [x] Port Spatial Queries (`point`, `segment`, `bb`, `shape` in `src/space/space_query.zig`)

- [ ] **Phase 6: Extras & Advanced Modules**
    - [x] Port Multithreaded Space (`src/extras/hasty_space.zig`: `cpHastySpace`)
    - [x] Port Marching Squares (`src/extras/march.zig`: `cpMarch`)
    - [x] Port Polyline Utilities (`src/extras/polyline.zig`: `cpPolyline`)
    - [x] Port Robust Geometry Helpers (`src/extras/robust.zig`: `cpRobust`)
    - [x] Port Debug Rendering Helpers (`src/extras/space_debug.zig`: `cpSpaceDebug`)

- [x] **Verification & Testing**
    - [x] Write Unit Tests for Math/Geometry
    - [x] Create Integration Tests/Demos
    - [x] (Optional) Visual Debugging Setup

---

## 1. Project Setup & Infrastructure

*   **Initialize Zig Project**:
    *   Run `zig init-lib` to create the project structure (already done in this repository; kept here for context).
    *   Configure `build.zig` to support library compilation and unit tests.
    *   Set up a "demo" or "benchmark" executable target to run verification simulations.
*   **Memory Management Strategy**:
    *   Replace Chipmunk's `cpcalloc`/`cpfree` macros with Zig's `std.mem.Allocator` interface.
    *   Pass allocators explicitly to functions that require memory allocation (e.g., `Space.init(allocator)`).

## 2. Phase 1: Foundation (Math & Core Types)

These are the leaf dependencies that everything else builds upon.

*   **Configuration & Core Types (`src/core/types.zig`)**:
    *   Define `cpFloat` (f32/f64) and `cpBool`.
    *   Port `cpCollisionType`, `cpGroup`, `cpBitmask`, IDs, and hash types.
*   **Vector Math (`src/core/vect.zig`)**:
    *   Port `cpVect` struct and operations (`cpvadd`, `cpvdot`, `cpvcross`, etc.).
    *   Internally, prefer using the SIMD-friendly [`zmath` library](ZMATH_API.md) (`F32x4`, `dot2`, `length2`, `normalize2`, etc.) to implement these operations, while keeping the public `cpVect` API and semantics identical to Chipmunk2D.
*   **Bounding Boxes (`src/core/bb.zig`)**:
    *   Port `cpBB` struct and operations (`intersects`, `contains`, `merge`).
*   **Transforms (`src/core/transform.zig`)**:
    *   Port `cpTransform` and any small matrix helpers used by the engine.
    *   Where appropriate (e.g. composing or interpolating transforms), leverage `zmath` matrix APIs (`Mat`, `rotationX/Y/Z`, `translation`, `mul`, etc.) as the implementation, keeping the observable 2D transform behavior consistent with Chipmunk2D.

## 3. Phase 2: Physics Primitives

Core structures that define physical objects.

*   **Rigid Bodies (`src/space/body.zig`)**:
    *   Port `cpBody` struct.
    *   Implement integration functions (`updateVelocity`, `updatePosition`).
    *   Handle mass property calculations.
*   **Shapes (`src/shape/*.zig`)**:
    *   Define the `Shape` interface/base type in `shape/shape_base.zig`.
    *   Port `cpShapeFilter`.
    *   Implement `CircleShape`, `SegmentShape`, and `PolyShape` in their own modules.
    *   *Note*: Consider using a tagged union for shapes if the set of shapes is closed, or an interface pattern if extensibility is required. Chipmunk allows custom shapes, but they are rare.
*   **Constraints Base (`src/constraint/constraint_base.zig`)**:
    *   Define the `Constraint` interface/base type.
    *   Port `cpConstraint` base logic (bias, error correction).

## 4. Phase 3: Spatial Indexing & Collision Detection

The "Broad Phase" and "Narrow Phase" of the simulation.

*   **Spatial Indexes (`src/spatial_index/*.zig`)**:
    *   Define the spatial index abstraction in `spatial_index/interface.zig`.
    *   Port `cpBBTree` (Bounding Box Tree) - this is the default and most important.
    *   (Optional) Port `cpSpaceHash` and `cpSweep1D` later.
*   **Collision Detection (`src/collision/collision.zig`)**:
    *   Port GJK (Gilbert-Johnson-Keerthi) and EPA (Expanding Polytope Algorithm).
    *   Port primitive collision tests (`CircleToCircle`, `SegmentToSegment`, etc.).
    *   Implement `cpCollide` dispatch logic.
*   **Arbiters (`src/collision/arbiter.zig`)**:
    *   Port `cpArbiter` struct.
    *   Implement contact point management and persistence.

## 5. Phase 4: Constraints & Joints

Porting the specific joint implementations.

*   **Simple Joints**: `PinJoint`, `SlideJoint`, `PivotJoint`.
*   **Complex Joints**: `GrooveJoint`, `DampedSpring`, `DampedRotarySpring`.
*   **Motors**: `SimpleMotor`, `GearJoint`, `RatchetJoint`, `RotaryLimitJoint`.

## 6. Phase 5: The Space (Simulation Core)

Putting it all together.

*   **Space (`src/space/*.zig`)**:
    *   Port `cpSpace` struct (`space.zig`).
    *   Implement `addBody`, `addShape`, `addConstraint`.
    *   Implement the `step` function (the main simulation loop) in `space_step.zig`.
    *   Port the Solver (impulse resolution).
    *   Implement callbacks (collision handlers).
*   **Queries (`src/space/space_query.zig`)**:
    *   Port `pointQuery`, `segmentQuery`, `bbQuery`, `shapeQuery`.

## 7. Phase 6: Extras & Advanced Modules

These map to Chipmunk’s non-core modules and should be implemented after the main engine is stable but before declaring the migration complete.

*   **Multithreaded Space (`src/extras/hasty_space.zig`)**:
    *   Port `cpHastySpace` and adapt threading to Zig (`std.Thread`).
*   **Marching Squares (`src/extras/march.zig`)**:
    *   Port `cpMarch` functionality.
*   **Polyline Utilities (`src/extras/polyline.zig`)**:
    *   Port `cpPolyline` and related helpers.
*   **Robust Geometry (`src/extras/robust.zig`)**:
    *   Port `cpRobust` functionality.
*   **Debug Rendering Helpers (`src/extras/space_debug.zig`)**:
    *   Port `cpSpaceDebug` for debug drawing integrations.

## 8. Verification & Testing

*   **Unit Tests**: Write Zig `test` blocks for math, geometry, and basic physics behavior.
*   **Integration Tests**: Create simple scenarios (e.g., a falling ball, a pendulum) and assert expected positions.
*   **Cross-Implementation Tests**: Where practical, compare Zig results to the original C implementation (via `@cImport`) to ensure behavior parity within a small tolerance.
*   **Visual Debugging**: (Optional) Bind to a simple renderer (like Raylib or just ASCII output) to visualize the simulation for manual verification.

## Key Zig Implementation Details

*   **Error Handling**: Use Zig's error unions for fallible operations (e.g., allocation failures).
*   **Pointers**: Use `*T` for mutable references and `*const T` for immutable ones.
*   **Arrays & Collections**: Use `std.ArrayList` and other Zig stdlib containers instead of Chipmunk's custom `cpArray`.
*   **Hash Sets/Maps**: Use `std.AutoHashMap` or `std.HashMap` instead of `cpHashSet`.
