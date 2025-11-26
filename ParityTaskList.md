# Parity Remediation Plan

This plan captures the open parity, performance, and memory items needed to bring the Zig port in line with Chipmunk2D behavior and semantics.

## Task Checklist

- [x] **Collision Handling**
    - [x] Implement pair-based handler lookup with wildcard precedence (typeA/typeB key) instead of single-type matching.
    - [x] Fire `separate` callbacks only on contact exit; avoid per-step separate calls while bodies still overlap.
- [x] **Sleeping & Contact Persistence**
    - [x] Restore idle-based sleeping using stamps and thresholds (`sleepTimeThreshold`/`idleSpeedThreshold`) rather than instantaneous energy cutoff.
    - [x] Keep arbiters alive for `collisionPersistence` frames to preserve warm-start data; avoid pruning after one missed frame.
- [x] **Collision Parameters**
    - [x] Make `collisionSlop` and `collisionBias` configurable per space with Chipmunk defaults; remove hard-coded values in arbiter setup.
- [x] **Geometry & Messaging**
    - [x] Align `cpAreaForPoly`/mass info for 2-vertex polygons with segment math (non-zero area/mass).
    - [x] Honor `is_hard_error` semantics in `cpMessage` (and platform logging hooks) or document the intentional divergence.
- [x] **API Surface Parity**
    - [x] Add helper entry points for iteration and queries with Zig-style callbacks and context pointers as equivalents to the Objective-C `_b` APIs.
- [x] **Memory & Per-Step Allocation**
    - [x] Reuse pooled storage for component/sleep processing and contact data; reduce per-step allocator churn to match Chipmunk’s pooled design.

## Detailed Tasks

### 1. Collision Handling

*   **Handler dispatch**: Build handler lookup keyed by `(typeA, typeB)` with wildcard precedence; cache results in `cpSpace` to mirror Chipmunk handler selection.
*   **Separation semantics**: Trigger `separate` only when a contact pair is removed from the cache; guard against repeated calls while overlapping.

### 2. Sleeping & Contact Persistence

*   **Idle-based sleep**: Integrate stamp/threshold-driven sleep using `sleepTimeThreshold` and `idleSpeedThreshold`; remove immediate-sleep path based solely on energy.
*   **Arbiter lifetime**: Retain arbiters for `collisionPersistence` frames; clear only after expiry and fire “separate” on eviction.

### 3. Collision Parameters

*   **Configurable slop/bias**: Expose `collisionSlop` and `collisionBias` on `cpSpace` with Chipmunk defaults; feed them into arbiter bias/slop calculations.

### 4. Geometry & Messaging

*   **Degenerate polys**: For 2-vertex polys, reuse segment area/mass computation rather than returning zero.
*   **cpMessage parity**: Respect `is_hard_error` (and optional platform logging) or document the intentional deviation clearly.

### 5. API Surface Parity

*   **Block helpers**: Implement `_b` helpers (`cpSpaceEach*`, queries) or provide documented Zig replacements to maintain API completeness.

### 6. Memory & Per-Step Allocation

*   **Pooling**: Replace per-step allocations in component/sleep processing with reusable buffers; ensure contact/arbiter data follows Chipmunk’s pool strategy.
