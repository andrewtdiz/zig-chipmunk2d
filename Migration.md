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