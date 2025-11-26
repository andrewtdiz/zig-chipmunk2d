# Chipmunk2D Parity Concerns

- [Resolved] `cpAreaForPoly` now reuses segment area for 2‑vertex polygons so degenerate polys report the same non-zero area/mass as segment math.
- [Resolved] Collision handlers now use pair keys with wildcard precedence and caching, matching Chipmunk’s `(typeA, typeB)` selection semantics.
- [Resolved] `separate` callbacks only fire when contacts leave the arbiter cache rather than every step of overlap.
- [Resolved] Sleeping uses idle-time and thresholds with component anchoring instead of instantaneous energy cutoffs.
- [Resolved] Arbiter entries persist for `collision_persistence` frames and defer eviction while contacts remain cached.
- [Resolved] Collision slop and bias are now space-configurable with Chipmunk defaults (`collision_slop = 0.1`, `collision_bias = pow(0.9, 60)`) feeding arbiter setup instead of hard-coded constants.
- [Resolved] Component/sleep processing now reuses pooled hash maps, arrays, and arbiter stale-key buffers each step to mirror Chipmunk’s contact and component pooling without allocator churn.
- [Resolved] Zig exposes helper callbacks for iterating bodies/shapes/constraints and querying spaces with user data, covering the Objective-C block helpers (`cpSpaceEachBody_b`, `cpSpacePointQuery_b`, etc.) with idiomatic equivalents.
- [Resolved] `cpMessage` now panics when flagged as a hard error, matching Chipmunk’s fatal semantics while retaining the existing stderr logging.
