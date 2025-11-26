const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const shape_base = @import("../shape/shape_base.zig");
const circle = @import("../shape/circle.zig");
const segment = @import("../shape/segment.zig");
const poly = @import("../shape/poly.zig");
const body_mod = @import("body.zig");
const constraint_base = @import("../constraint/constraint_base.zig");
const joints = @import("../constraint/joints.zig");
const collision = @import("../collision/collision.zig");
const arbiter = @import("../collision/arbiter.zig");
const spatial_interface = @import("../spatial_index/interface.zig");
const bbtree = @import("../spatial_index/bbtree.zig");
const space_hash_mod = @import("../spatial_index/space_hash.zig");
const sweep_mod = @import("../spatial_index/sweep1d.zig");
const step_module = @import("space_step.zig");
const query_module = @import("space_query.zig");

pub const ConstraintOps = struct {
    preStep: ?fn (*anyopaque, types.cpFloat) void = null,
    applyCachedImpulse: ?fn (*anyopaque, types.cpFloat) void = null,
    applyImpulse: ?fn (*anyopaque) void = null,
    postStep: ?fn (*anyopaque) void = null,
};

pub const ConstraintCallbackPhase = enum {
    preStep,
    applyCachedImpulse,
    applyImpulse,
    postStep,
};

pub const ConstraintEntry = struct {
    constraint: *constraint_base.cpConstraint,
    payload: *anyopaque,
    ops: ConstraintOps,
};

pub const PostStepCallback = struct {
    key: ?*const anyopaque,
    func: fn (*cpSpace, *anyopaque) void,
    data: *anyopaque,
};

pub const CollisionHandler = struct {
    begin: ?fn (*arbiter.cpArbiter, *cpSpace) types.cpBool = null,
    preSolve: ?fn (*arbiter.cpArbiter, *cpSpace) types.cpBool = null,
    postSolve: ?fn (*arbiter.cpArbiter, *cpSpace) void = null,
    separate: ?fn (*arbiter.cpArbiter, *cpSpace) void = null,
};

pub const IndexKind = enum { bb_tree, space_hash, sweep1d };

const IndexUnion = union(IndexKind) {
    bb_tree: bbtree.cpBBTree,
    space_hash: space_hash_mod.cpSpaceHash,
    sweep1d: sweep_mod.cpSweep1D,
};

pub const ManagedIndex = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const spatial_interface.BoundsFunc,
    context: ?*const anyopaque,
    kind: IndexKind,
    storage: IndexUnion,

    pub fn init(
        allocator: std.mem.Allocator,
        kind: IndexKind,
        bounds_func: *const spatial_interface.BoundsFunc,
        context: ?*const anyopaque,
        config: ?space_hash_mod.Config,
    ) ManagedIndex {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .kind = kind,
            .storage = switch (kind) {
                .bb_tree => IndexUnion{ .bb_tree = bbtree.cpBBTree.init(allocator, bounds_func, context) },
                .space_hash => IndexUnion{ .space_hash = space_hash_mod.cpSpaceHash.init(allocator, bounds_func, context, config orelse .{}) },
                .sweep1d => IndexUnion{ .sweep1d = sweep_mod.cpSweep1D.init(allocator, bounds_func, context) },
            },
        };
    }

    pub fn deinit(self: *ManagedIndex) void {
        switch (self.kind) {
            .bb_tree => self.storage.bb_tree.deinit(),
            .space_hash => self.storage.space_hash.deinit(),
            .sweep1d => self.storage.sweep1d.deinit(),
        }
    }

    pub fn insert(self: *ManagedIndex, object: *const anyopaque) !void {
        switch (self.kind) {
            .bb_tree => try self.storage.bb_tree.insert(object),
            .space_hash => try self.storage.space_hash.insert(object),
            .sweep1d => try self.storage.sweep1d.insert(object),
        }
    }

    pub fn remove(self: *ManagedIndex, object: *const anyopaque) void {
        switch (self.kind) {
            .bb_tree => self.storage.bb_tree.remove(object),
            .space_hash => self.storage.space_hash.remove(object),
            .sweep1d => self.storage.sweep1d.remove(object),
        }
    }

    pub fn reindex(self: *ManagedIndex) !void {
        switch (self.kind) {
            .bb_tree => try self.storage.bb_tree.reindex(),
            .space_hash => try self.storage.space_hash.reindex(),
            .sweep1d => try self.storage.sweep1d.reindex(),
        }
    }

    pub fn query(self: *ManagedIndex, bounds: bb.cpBB, func: *const spatial_interface.QueryFunc, data: ?*anyopaque) void {
        switch (self.kind) {
            .bb_tree => self.storage.bb_tree.query(bounds, func, data),
            .space_hash => self.storage.space_hash.query(bounds, func, data),
            .sweep1d => self.storage.sweep1d.query(bounds, func, data),
        }
    }

    pub fn each(self: *ManagedIndex, func: *const spatial_interface.EachFunc, data: ?*anyopaque) void {
        switch (self.kind) {
            .bb_tree => self.storage.bb_tree.each(func, data),
            .space_hash => self.storage.space_hash.each(func, data),
            .sweep1d => self.storage.sweep1d.each(func, data),
        }
    }

    pub fn count(self: ManagedIndex) usize {
        return switch (self.kind) {
            .bb_tree => self.storage.bb_tree.count(),
            .space_hash => self.storage.space_hash.count(),
            .sweep1d => self.storage.sweep1d.count(),
        };
    }
};

const ConstraintRuntimeStorage = struct {
    allocator: std.mem.Allocator,
    slots: std.AutoHashMapUnmanaged(*constraint_base.cpConstraint, Slot) = .{},

    const Slot = struct { buffer: []u8 };

    pub fn init(allocator: std.mem.Allocator) ConstraintRuntimeStorage {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *ConstraintRuntimeStorage) void {
        var it = self.slots.iterator();
        while (it.next()) |entry| {
            self.allocator.free(entry.value_ptr.buffer);
        }
        self.slots.deinit(self.allocator);
    }

    pub fn ensure(self: *ConstraintRuntimeStorage, constraint: *constraint_base.cpConstraint, size: usize) ![]u8 {
        if (self.slots.getPtr(constraint)) |slot| {
            if (slot.buffer.len < size) {
                self.allocator.free(slot.buffer);
                slot.buffer = try self.allocator.alloc(u8, size);
            }
            return slot.buffer;
        }

        const buffer = try self.allocator.alloc(u8, size);
        try self.slots.put(self.allocator, constraint, .{ .buffer = buffer });
        return buffer;
    }

    pub fn get(self: *ConstraintRuntimeStorage, constraint: *constraint_base.cpConstraint) ?[]u8 {
        if (self.slots.getPtr(constraint)) |slot| {
            return slot.buffer;
        }
        return null;
    }

    pub fn release(self: *ConstraintRuntimeStorage, constraint: *constraint_base.cpConstraint) void {
        if (self.slots.remove(constraint)) |slot| {
            self.allocator.free(slot.buffer);
        }
    }
};

const DEFAULT_COLLISION_SLOP: types.cpFloat = 0.1;
const DEFAULT_COLLISION_BIAS: types.cpFloat = types.cpfpow(1.0 - 0.1, 60.0);

fn shapeBounds(ptr: *const anyopaque, _: ?*const anyopaque) bb.cpBB {
    const shape = @as(*const shape_base.cpShape, @ptrCast(ptr));
    return shape.bbValue();
}

const CachedArbiter = struct {
    value: arbiter.cpArbiter,
    stamp: u64,
    separated: bool = false,
};

pub const cpSpace = struct {
    allocator: std.mem.Allocator,
    gravity: vect.cpVect = vect.cpvzero,
    damping: types.cpFloat = 1.0,
    iterations: usize = 10,
    stamp: u64 = 0,
    prev_dt: types.cpFloat = 0.0,
    curr_dt: types.cpFloat = 0.0,
    sleep_time_threshold: types.cpFloat = types.CP_INFINITY,
    idle_speed_threshold: types.cpFloat = 0.0,
    collision_persistence: types.cpTimestamp = 3,
    collision_slop: types.cpFloat = DEFAULT_COLLISION_SLOP,
    collision_bias: types.cpFloat = DEFAULT_COLLISION_BIAS,

    bodies: std.ArrayList(*body_mod.cpBody),
    shapes: std.ArrayList(*shape_base.cpShape),
    dynamic_shapes: std.ArrayList(*shape_base.cpShape),
    static_shapes: std.ArrayList(*shape_base.cpShape),
    constraints: std.ArrayList(ConstraintEntry),
    arbiters: std.ArrayList(arbiter.cpArbiter),
    arbiter_cache: std.AutoHashMap(u128, CachedArbiter),
    arbiter_pool: std.ArrayList(arbiter.cpArbiter),
    post_steps: std.ArrayList(PostStepCallback),
    collision_cache: collision.CollisionIdCache,
    pair_handlers: std.AutoHashMap(u128, CollisionHandler),
    handler_cache: std.AutoHashMap(u128, CollisionHandler),
    constraint_runtime: ConstraintRuntimeStorage,
    handlers: std.AutoHashMap(types.cpCollisionType, CollisionHandler),
    wildcard_handlers: std.AutoHashMap(types.cpCollisionType, CollisionHandler),
    handler: CollisionHandler = .{},
    dynamic_index: ManagedIndex,
    static_index: ManagedIndex,
    component_index_map: std.AutoHashMap(*body_mod.cpBody, usize),
    component_parents: std.ArrayList(usize),
    component_ranks: std.ArrayList(u8),
    component_body_anchored: std.ArrayList(bool),
    component_root_anchored: std.ArrayList(bool),
    component_min_idle: std.ArrayList(types.cpFloat),
    stale_arbiter_keys: std.ArrayList(u128),

    pub fn init(allocator: std.mem.Allocator) cpSpace {
        return .{
            .allocator = allocator,
            .bodies = .empty,
            .shapes = .empty,
            .dynamic_shapes = .empty,
            .static_shapes = .empty,
            .constraints = .empty,
            .arbiters = .empty,
            .arbiter_cache = std.AutoHashMap(u128, CachedArbiter).init(allocator),
            .arbiter_pool = .empty,
            .post_steps = .empty,
            .collision_cache = collision.CollisionIdCache.init(allocator),
            .pair_handlers = std.AutoHashMap(u128, CollisionHandler).init(allocator),
            .handler_cache = std.AutoHashMap(u128, CollisionHandler).init(allocator),
            .constraint_runtime = ConstraintRuntimeStorage.init(allocator),
            .handlers = std.AutoHashMap(types.cpCollisionType, CollisionHandler).init(allocator),
            .wildcard_handlers = std.AutoHashMap(types.cpCollisionType, CollisionHandler).init(allocator),
            .dynamic_index = ManagedIndex.init(allocator, .bb_tree, shapeBounds, null, null),
            .static_index = ManagedIndex.init(allocator, .bb_tree, shapeBounds, null, null),
            .component_index_map = std.AutoHashMap(*body_mod.cpBody, usize).init(allocator),
            .component_parents = .empty,
            .component_ranks = .empty,
            .component_body_anchored = .empty,
            .component_root_anchored = .empty,
            .component_min_idle = .empty,
            .stale_arbiter_keys = .empty,
        };
    }

    pub fn deinit(self: *cpSpace) void {
        self.constraint_runtime.deinit();
        self.collision_cache.deinit();
        self.pair_handlers.deinit();
        self.handler_cache.deinit();
        self.bodies.deinit(self.allocator);
        self.shapes.deinit(self.allocator);
        self.dynamic_shapes.deinit(self.allocator);
        self.static_shapes.deinit(self.allocator);
        self.constraints.deinit(self.allocator);
        self.arbiters.deinit(self.allocator);
        self.arbiter_cache.deinit();
        self.arbiter_pool.deinit(self.allocator);
        self.post_steps.deinit(self.allocator);
        self.handlers.deinit();
        self.wildcard_handlers.deinit();
        self.dynamic_index.deinit();
        self.static_index.deinit();
        self.component_index_map.deinit();
        self.component_parents.deinit(self.allocator);
        self.component_ranks.deinit(self.allocator);
        self.component_body_anchored.deinit(self.allocator);
        self.component_root_anchored.deinit(self.allocator);
        self.component_min_idle.deinit(self.allocator);
        self.stale_arbiter_keys.deinit(self.allocator);
    }

    pub fn addBody(self: *cpSpace, body: *body_mod.cpBody) !void {
        body.sleeping = false;
        body.idle_stamp = self.stamp;
        body.idle_time = 0.0;
        body.shape_list = null;
        body.constraint_list = null;
        body.arbiter_list = null;
        try self.bodies.append(self.allocator, body);
    }

    pub fn removeBody(self: *cpSpace, body: *body_mod.cpBody) void {
        removePtr(*body_mod.cpBody, &self.bodies, body);
        filterArbitersWithBody(self, body);
        var i: usize = 0;
        while (i < self.shapes.items.len) : (i += 1) {
            const shape = self.shapes.items[i];
            if (shape.body == body) {
                self.removeShape(shape);
                if (i > 0) i -= 1;
            }
        }
        var c: usize = 0;
        while (c < self.constraints.items.len) : (c += 1) {
            if (self.constraints.items[c].constraint.a == body or self.constraints.items[c].constraint.b == body) {
                const removed = self.constraints.orderedRemove(c);
                removed.constraint.a.detachConstraint(removed.constraint);
                removed.constraint.b.detachConstraint(removed.constraint);
                self.constraint_runtime.release(removed.constraint);
                c -= 1;
            }
        }
    }

    pub fn addShape(self: *cpSpace, shape: *shape_base.cpShape) !void {
        try self.shapes.append(self.allocator, shape);
        shape.body.attachShape(shape);
        cacheShapeInternal(shape);
        const ptr = @as(*const anyopaque, @ptrCast(shape));
        switch (shape.body.body_type) {
            .static => {
                try self.static_shapes.append(self.allocator, shape);
                try self.static_index.insert(ptr);
            },
            else => {
                try self.dynamic_shapes.append(self.allocator, shape);
                try self.dynamic_index.insert(ptr);
            },
        }
    }

    pub fn removeShape(self: *cpSpace, shape: *shape_base.cpShape) void {
        removePtr(*shape_base.cpShape, &self.shapes, shape);
        shape.body.detachShape(shape);
        const ptr = @as(*const anyopaque, @ptrCast(shape));
        self.collision_cache.removeShape(shape);
        filterArbitersWithShape(self, shape);
        switch (shape.body.body_type) {
            .static => {
                removePtr(*shape_base.cpShape, &self.static_shapes, shape);
                self.static_index.remove(ptr);
            },
            else => {
                removePtr(*shape_base.cpShape, &self.dynamic_shapes, shape);
                self.dynamic_index.remove(ptr);
            },
        }
    }

    pub fn addConstraint(self: *cpSpace, constraint: *constraint_base.cpConstraint, ops: ConstraintOps, payload: ?*anyopaque) !void {
        const stored_payload: *anyopaque = payload orelse @as(*anyopaque, @ptrCast(constraint));
        try self.constraints.append(self.allocator, .{ .constraint = constraint, .payload = stored_payload, .ops = ops });
        constraint.a.attachConstraint(constraint);
        if (constraint.b != constraint.a) {
            constraint.b.attachConstraint(constraint);
        }
    }

    pub fn removeConstraint(self: *cpSpace, constraint: *constraint_base.cpConstraint) void {
        var i: usize = 0;
        while (i < self.constraints.items.len) : (i += 1) {
            if (self.constraints.items[i].constraint == constraint) {
                constraint.a.detachConstraint(constraint);
                constraint.b.detachConstraint(constraint);
                _ = self.constraints.orderedRemove(i);
                self.constraint_runtime.release(constraint);
                break;
            }
        }
    }

    pub fn ensureConstraintStorage(self: *cpSpace, constraint: *constraint_base.cpConstraint, size: usize) ![]u8 {
        return self.constraint_runtime.ensure(constraint, size);
    }

    pub fn constraintStorage(self: *cpSpace, constraint: *constraint_base.cpConstraint) ?[]u8 {
        return self.constraint_runtime.get(constraint);
    }

    pub fn addPostStep(self: *cpSpace, key: ?*const anyopaque, func: fn (*cpSpace, *anyopaque) void, data: *anyopaque) !void {
        try self.post_steps.append(self.allocator, .{ .key = key, .func = func, .data = data });
    }

    pub fn setCollisionHandler(self: *cpSpace, handler: CollisionHandler) void {
        self.handler = handler;
        self.handler_cache.clearRetainingCapacity();
    }

    pub fn setCollisionHandlerForPair(
        self: *cpSpace,
        type_a: types.cpCollisionType,
        type_b: types.cpCollisionType,
        handler: CollisionHandler,
    ) void {
        const key = makeTypePairKey(type_a, type_b);
        self.pair_handlers.put(self.allocator, key, handler) catch {};
        self.handler_cache.clearRetainingCapacity();
    }

    pub fn setCollisionHandlerForType(self: *cpSpace, collision_type: types.cpCollisionType, handler: CollisionHandler) void {
        self.handlers.put(self.allocator, collision_type, handler) catch {};
        self.handler_cache.clearRetainingCapacity();
    }

    pub fn setWildcardHandler(self: *cpSpace, collision_type: types.cpCollisionType, handler: CollisionHandler) void {
        self.wildcard_handlers.put(self.allocator, collision_type, handler) catch {};
        self.handler_cache.clearRetainingCapacity();
    }

    pub fn wakeBody(self: *cpSpace, body: *body_mod.cpBody) void {
        body.sleeping = false;
        body.idle_stamp = self.stamp;
        body.idle_time = 0.0;
    }

    pub fn activateBody(self: *cpSpace, body: *body_mod.cpBody) void {
        self.wakeBody(body);
        if (body.body_type != .dynamic) return;

        body.v_bias = vect.cpvzero;
        body.w_bias = 0.0;

        var arb = body.arbiter_list;
        while (arb) |entry| {
            const other = entry.otherBody(body);
            if (other.body_type == .dynamic and other.sleeping) {
                self.wakeBody(other);
            }
            arb = entry.nextForBody(body);
        }

        var constraint_ptr = body.constraint_list;
        while (constraint_ptr) |c| {
            const other = if (c.a == body) c.b else c.a;
            if (other.body_type == .dynamic and other.sleeping) {
                self.wakeBody(other);
            }
            constraint_ptr = c.nextForBody(body);
        }
    }

    pub fn step(self: *cpSpace, dt: types.cpFloat) void {
        Stepper.step(self, dt);
    }

    pub fn pointQuery(
        self: *const cpSpace,
        point: vect.cpVect,
        filter: shape_base.cpShapeFilter,
        func: fn (*shape_base.cpShape, vect.cpVect, types.cpFloat, ?*anyopaque) void,
        data: ?*anyopaque,
    ) void {
        QueryAPI.pointQuery(self, point, filter, func, data);
    }

    pub fn bbQuery(
        self: *const cpSpace,
        bounds: bb.cpBB,
        filter: shape_base.cpShapeFilter,
        func: fn (*shape_base.cpShape, ?*anyopaque) void,
        data: ?*anyopaque,
    ) void {
        QueryAPI.bbQuery(self, bounds, filter, func, data);
    }

    pub fn segmentQuery(
        self: *const cpSpace,
        start: vect.cpVect,
        end: vect.cpVect,
        radius: types.cpFloat,
        filter: shape_base.cpShapeFilter,
        func: fn (*shape_base.cpShape, vect.cpVect, vect.cpVect, types.cpFloat, ?*anyopaque) void,
        data: ?*anyopaque,
    ) void {
        QueryAPI.segmentQuery(self, start, end, radius, filter, func, data);
    }

    pub fn segmentQueryFirst(
        self: *const cpSpace,
        start: vect.cpVect,
        end: vect.cpVect,
        radius: types.cpFloat,
        filter: shape_base.cpShapeFilter,
        info: ?*shape_base.cpSegmentQueryInfo,
    ) ?*shape_base.cpShape {
        return QueryAPI.segmentQueryFirst(self, start, end, radius, filter, info);
    }

    pub fn shapeQuery(
        self: *const cpSpace,
        target: *shape_base.cpShape,
        func: fn (*shape_base.cpShape, collision.CollisionResult, ?*anyopaque) void,
        data: ?*anyopaque,
    ) void {
        QueryAPI.shapeQuery(self, target, func, data);
    }

    pub fn eachBody(self: *const cpSpace, func: fn (*body_mod.cpBody, ?*anyopaque) void, data: ?*anyopaque) void {
        for (self.bodies.items) |body| {
            func(body, data);
        }
    }

    pub fn eachShape(self: *const cpSpace, func: fn (*shape_base.cpShape, ?*anyopaque) void, data: ?*anyopaque) void {
        for (self.shapes.items) |shape| {
            func(shape, data);
        }
    }

    pub fn eachConstraint(self: *const cpSpace, func: fn (*constraint_base.cpConstraint, ?*anyopaque) void, data: ?*anyopaque) void {
        for (self.constraints.items) |entry| {
            func(entry.constraint, data);
        }
    }

    pub fn setDynamicIndex(self: *cpSpace, kind: IndexKind) !void {
        if (self.dynamic_index.kind == kind) return;
        try self.rebuildIndex(&self.dynamic_index, self.dynamic_shapes.items, kind, null);
    }

    pub fn setStaticIndex(self: *cpSpace, kind: IndexKind) !void {
        if (self.static_index.kind == kind) return;
        try self.rebuildIndex(&self.static_index, self.static_shapes.items, kind, null);
    }

    pub fn reindexStatic(self: *cpSpace) void {
        self.static_index.reindex() catch {};
    }

    pub fn useSpatialHash(self: *cpSpace, dim: types.cpFloat, count: usize) !void {
        const config: space_hash_mod.Config = .{ .cell_dim = dim, .max_cells = count };
        try self.rebuildIndex(&self.dynamic_index, self.dynamic_shapes.items, .space_hash, config);
        try self.rebuildIndex(&self.static_index, self.static_shapes.items, .space_hash, config);
    }

    pub fn reindexShape(self: *cpSpace, shape: *shape_base.cpShape) void {
        cacheShapeInternal(shape);
        const ptr = @as(*const anyopaque, @ptrCast(shape));
        switch (shape.body.body_type) {
            .static => {
                self.static_index.remove(ptr);
                self.static_index.insert(ptr) catch {};
            },
            else => {
                self.dynamic_index.remove(ptr);
                self.dynamic_index.insert(ptr) catch {};
            },
        }
    }

    pub fn reindexShapesForBody(self: *cpSpace, body: *body_mod.cpBody) void {
        var shape_ptr = body.shape_list;
        while (shape_ptr) |shape| {
            self.reindexShape(shape);
            shape_ptr = shape.next;
        }
    }
};

fn rebuildIndex(
    self: *cpSpace,
    index: *ManagedIndex,
    shapes: []const *shape_base.cpShape,
    kind: IndexKind,
    config: ?space_hash_mod.Config,
) !void {
    var replacement = ManagedIndex.init(self.allocator, kind, shapeBounds, null, config);
    errdefer replacement.deinit();
    for (shapes) |shape| {
        try replacement.insert(@as(*const anyopaque, @ptrCast(shape)));
    }
    index.deinit();
    index.* = replacement;
}
const Stepper = step_module.makeStepper(cpSpace, struct {
    pub fn cacheShape(shape: *shape_base.cpShape) void {
        cacheShapeInternal(shape);
    }
});

const QueryAPI = query_module.makeQueryApi(cpSpace);

fn removePtr(comptime T: type, list: *std.ArrayList(T), target: T) void {
    var i: usize = 0;
    while (i < list.items.len) : (i += 1) {
        if (list.items[i] == target) {
            _ = list.orderedRemove(i);
            break;
        }
    }
}

pub fn runConstraintCallback(constraints: []ConstraintEntry, dt: types.cpFloat, dt_coef: types.cpFloat, phase: ConstraintCallbackPhase) void {
    runConstraintCallbackRange(constraints, dt, dt_coef, phase, 0, constraints.len);
}

pub fn runConstraintCallbackRange(
    constraints: []ConstraintEntry,
    dt: types.cpFloat,
    dt_coef: types.cpFloat,
    phase: ConstraintCallbackPhase,
    start: usize,
    end: usize,
) void {
    var idx = start;
    while (idx < end) : (idx += 1) {
        const entry = constraints[idx];
        switch (phase) {
            .preStep => if (entry.ops.preStep) |fn_ptr| fn_ptr(entry.payload, dt),
            .applyCachedImpulse => if (entry.ops.applyCachedImpulse) |fn_ptr| fn_ptr(entry.payload, dt_coef),
            .applyImpulse => if (entry.ops.applyImpulse) |fn_ptr| fn_ptr(entry.payload),
            .postStep => if (entry.ops.postStep) |fn_ptr| fn_ptr(entry.payload),
        }
    }
}

pub fn updateVelocities(space: *cpSpace, dt: types.cpFloat) void {
    const damping_factor = std.math.pow(types.cpFloat, space.damping, dt);
    for (space.bodies.items) |body| {
        if (body.sleeping) continue;
        body.updateVelocity(space.gravity, damping_factor, dt);
    }
}

pub fn integratePositions(space: *cpSpace, dt: types.cpFloat) void {
    for (space.bodies.items) |body| {
        if (body.sleeping) continue;
        body.updatePosition(dt);
    }
}

pub fn updateShapeCaches(space: *cpSpace) void {
    for (space.shapes.items) |shape| {
        cacheShapeInternal(shape);
    }
}

pub fn resolveCollisions(space: *cpSpace) void {
    startBroadPhase(space);
    resolveCollisionsRange(space, 0, space.dynamic_shapes.items.len, null);
    finishBroadPhase(space);
}

pub fn processComponents(space: *cpSpace, dt: types.cpFloat) void {
    Stepper.processComponents(space, dt);
}

pub fn resolveArbiters(space: *cpSpace) void {
    resolveArbitersRange(space, 0, space.arbiters.items.len);
}

pub fn resolveArbitersRange(space: *cpSpace, start: usize, end: usize) void {
    var idx = start;
    while (idx < end) : (idx += 1) {
        var arb_ptr = &space.arbiters.items[idx];
        const bias_coef = constraint_base.biasCoefficient(space.collision_bias, 1.0);
        arb_ptr.preStep(1.0, space.collision_slop, bias_coef);
        arb_ptr.applyImpulse();
    }
}

pub fn postSolveArbiters(space: *cpSpace) void {
    for (space.arbiters.items) |*arb_ref| {
        const handler = selectHandler(space, arb_ref.shape_a, arb_ref.shape_b);
        if (handler.postSolve) |post_func| {
            post_func(arb_ref, space);
        }
    }
}

pub fn runPostSteps(space: *cpSpace) void {
    for (space.post_steps.items) |callback| {
        callback.func(space, callback.data);
    }
    space.post_steps.clearRetainingCapacity();
}

pub fn preStepArbiters(space: *cpSpace, dt: types.cpFloat) void {
    const bias_coef = constraint_base.biasCoefficient(space.collision_bias, dt);
    for (space.arbiters.items) |*arb_ref| {
        arb_ref.preStep(dt, space.collision_slop, bias_coef);
    }
}

pub fn applyCachedArbiterImpulses(space: *cpSpace, dt_coef: types.cpFloat) void {
    for (space.arbiters.items) |*arb_ref| {
        arb_ref.applyCachedImpulse(dt_coef);
    }
}

fn cacheShapeInternal(shape: *shape_base.cpShape) void {
    switch (shape.shape_type) {
        .circle => asCircle(shape).cacheBB(),
        .segment => asSegment(shape).cacheBB(),
        .poly => asPoly(shape).cacheBB(),
    }
}

fn unlinkArbiterFromBodies(arb_ptr: *arbiter.cpArbiter) void {
    arb_ptr.shape_a.body.detachArbiter(arb_ptr);
    arb_ptr.shape_b.body.detachArbiter(arb_ptr);
    arb_ptr.resetThreads();
}

fn relinkArbiter(space: *cpSpace, index: usize) void {
    var arb_ptr = &space.arbiters.items[index];
    arb_ptr.resetThreads();
    arb_ptr.shape_a.body.attachArbiter(arb_ptr);
    arb_ptr.shape_b.body.attachArbiter(arb_ptr);
}

fn removeArbiterAt(space: *cpSpace, index: usize) arbiter.cpArbiter {
    const last_idx = space.arbiters.items.len - 1;
    unlinkArbiterFromBodies(&space.arbiters.items[index]);
    const removed = space.arbiters.items[index];
    if (index != last_idx) {
        unlinkArbiterFromBodies(&space.arbiters.items[last_idx]);
        space.arbiters.items[index] = space.arbiters.items[last_idx];
        relinkArbiter(space, index);
    }
    space.arbiters.items.len -= 1;
    return removed;
}

fn attachArbitersToBodies(space: *cpSpace) void {
    for (space.bodies.items) |body| {
        body.arbiter_list = null;
    }
    var i: usize = 0;
    while (i < space.arbiters.items.len) : (i += 1) {
        relinkArbiter(space, i);
    }

    var it = space.arbiter_cache.iterator();
    while (it.next()) |entry| {
        if (entry.value_ptr.stamp == space.stamp) continue;
        if (entry.value_ptr.separated) continue;

        var cached = &entry.value_ptr.value;
        cached.resetThreads();
        cached.shape_a.body.attachArbiter(cached);
        cached.shape_b.body.attachArbiter(cached);
    }
}

fn recycleArbiters(space: *cpSpace) void {
    while (space.arbiters.items.len > 0) {
        const removed = removeArbiterAt(space, space.arbiters.items.len - 1);
        space.arbiter_pool.append(space.allocator, removed) catch {};
    }
}

pub fn startBroadPhase(space: *cpSpace) void {
    space.stamp +%= 1;
    recycleArbiters(space);
    space.dynamic_index.reindex() catch {};
}

pub fn resolveCollisionsRange(space: *cpSpace, start: usize, end: usize, mutex: ?*std.Thread.Mutex) void {
    var idx = start;
    while (idx < end) : (idx += 1) {
        const shape = space.dynamic_shapes.items[idx];
        var ctx = BroadPhaseContext{
            .space = space,
            .primary = shape,
            .static_query = false,
            .mutex = mutex,
        };
        space.dynamic_index.query(shape.bbValue(), queryPairs, &ctx);
        ctx.static_query = true;
        space.static_index.query(shape.bbValue(), queryPairs, &ctx);
    }
}

pub fn finishBroadPhase(space: *cpSpace) void {
    attachArbitersToBodies(space);
    syncArbiterCache(space);
    pruneArbiterCache(space);
}

fn takeArbiter(space: *cpSpace, shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) arbiter.cpArbiter {
    var idx: usize = 0;
    while (idx < space.arbiter_pool.items.len) : (idx += 1) {
        if (space.arbiter_pool.items[idx].matches(shape_a, shape_b)) {
            var pooled = space.arbiter_pool.swapRemove(idx);
            pooled.reuse(shape_a, shape_b);
            return pooled;
        }
    }
    return arbiter.cpArbiter.init(shape_a, shape_b);
}

fn stashArbiter(space: *cpSpace, arb: arbiter.cpArbiter) void {
    var cleaned = arb;
    cleaned.resetThreads();
    space.arbiter_pool.append(space.allocator, cleaned) catch {};
}

fn makeTypePairKey(type_a: types.cpCollisionType, type_b: types.cpCollisionType) u128 {
    const min_type = @min(type_a, type_b);
    const max_type = @max(type_a, type_b);
    return (@as(u128, min_type) << 64) | @as(u128, max_type);
}

fn selectHandler(space: *cpSpace, a: *shape_base.cpShape, b: *shape_base.cpShape) CollisionHandler {
    const key = makeTypePairKey(a.collision_type, b.collision_type);
    if (space.handler_cache.get(key)) |cached| return cached;

    if (space.pair_handlers.get(key)) |handler| {
        space.handler_cache.put(space.allocator, key, handler) catch {};
        return handler;
    }

    if (space.wildcard_handlers.get(a.collision_type)) |handler| {
        space.handler_cache.put(space.allocator, key, handler) catch {};
        return handler;
    }

    if (space.wildcard_handlers.get(b.collision_type)) |handler| {
        space.handler_cache.put(space.allocator, key, handler) catch {};
        return handler;
    }

    if (space.handlers.get(a.collision_type)) |handler| {
        space.handler_cache.put(space.allocator, key, handler) catch {};
        return handler;
    }

    if (space.handlers.get(b.collision_type)) |handler| {
        space.handler_cache.put(space.allocator, key, handler) catch {};
        return handler;
    }

    space.handler_cache.put(space.allocator, key, space.handler) catch {};
    return space.handler;
}

fn makePairKey(a: *const shape_base.cpShape, b: *const shape_base.cpShape) u128 {
    const first = @intFromPtr(a);
    const second = @intFromPtr(b);
    const min_ptr = @min(first, second);
    const max_ptr = @max(first, second);
    return (@as(u128, min_ptr) << 64) | @as(u128, max_ptr);
}

fn fetchArbiter(space: *cpSpace, a: *shape_base.cpShape, b: *shape_base.cpShape) ?*CachedArbiter {
    const key = makePairKey(a, b);
    var gop = space.arbiter_cache.getOrPut(key) catch return null;
    if (!gop.found_existing) {
        const pooled = takeArbiter(space, a, b);
        gop.value_ptr.* = .{ .value = pooled, .stamp = space.stamp };
    }
    gop.value_ptr.stamp = space.stamp;
    gop.value_ptr.separated = false;
    gop.value_ptr.value.reuse(a, b);
    return gop.value_ptr;
}

fn activatePair(space: *cpSpace, a: *shape_base.cpShape, b: *shape_base.cpShape) void {
    space.activateBody(a.body);
    space.activateBody(b.body);
}

fn syncArbiterCache(space: *cpSpace) void {
    for (space.arbiters.items) |arb_ref| {
        var copy = arb_ref;
        copy.resetThreads();
        const key = makePairKey(copy.shape_a, copy.shape_b);
        if (space.arbiter_cache.getPtr(key)) |cached| {
            cached.value = copy;
            cached.stamp = space.stamp;
            cached.separated = false;
        }
    }
}

fn pruneArbiterCache(space: *cpSpace) void {
    space.stale_arbiter_keys.clearRetainingCapacity();

    var it = space.arbiter_cache.iterator();
    while (it.next()) |entry| {
        const body_a = entry.value_ptr.value.shape_a.body;
        const body_b = entry.value_ptr.value.shape_b.body;
        if ((body_a.body_type == .static or body_a.sleeping) and (body_b.body_type == .static or body_b.sleeping)) {
            continue;
        }

        const ticks = space.stamp - entry.value_ptr.stamp;
        if (ticks >= 1 and !entry.value_ptr.separated) {
            const handler = selectHandler(space, entry.value_ptr.value.shape_a, entry.value_ptr.value.shape_b);
            if (handler.separate) |sep_func| {
                var temp = entry.value_ptr.value;
                sep_func(&temp, space);
            }
            entry.value_ptr.separated = true;
        }

        if (ticks >= @as(u64, space.collision_persistence)) {
            space.stale_arbiter_keys.append(space.allocator, entry.key_ptr.*) catch {};
        }
    }

    for (space.stale_arbiter_keys.items) |key| {
        if (space.arbiter_cache.remove(key)) |cached| {
            stashArbiter(space, cached.value);
        }
    }
}

fn matchesShape(arb_ref: arbiter.cpArbiter, shape: *shape_base.cpShape) bool {
    return arb_ref.shape_a == shape or arb_ref.shape_b == shape;
}

fn matchesBody(arb_ref: arbiter.cpArbiter, body: *body_mod.cpBody) bool {
    return arb_ref.shape_a.body == body or arb_ref.shape_b.body == body;
}

fn filterArbitersWithShape(space: *cpSpace, shape: *shape_base.cpShape) void {
    var i: usize = 0;
    while (i < space.arbiters.items.len) {
        const handler = selectHandler(space, space.arbiters.items[i].shape_a, space.arbiters.items[i].shape_b);
        if (matchesShape(space.arbiters.items[i], shape)) {
            if (handler.separate) |sep_func| {
                var temp = space.arbiters.items[i];
                sep_func(&temp, space);
            }
            const removed = removeArbiterAt(space, i);
            stashArbiter(space, removed);
            continue;
        }
        i += 1;
    }

    space.stale_arbiter_keys.clearRetainingCapacity();

    var it = space.arbiter_cache.iterator();
    while (it.next()) |entry| {
        if (matchesShape(entry.value_ptr.value, shape)) {
            space.stale_arbiter_keys.append(space.allocator, entry.key_ptr.*) catch {};
        }
    }

    for (space.stale_arbiter_keys.items) |key| {
        if (space.arbiter_cache.remove(key)) |cached| {
            const handler = selectHandler(space, cached.value.shape_a, cached.value.shape_b);
            if (handler.separate) |sep_func| {
                var temp = cached.value;
                sep_func(&temp, space);
            }
            stashArbiter(space, cached.value);
        }
    }
}

fn filterArbitersWithBody(space: *cpSpace, body: *body_mod.cpBody) void {
    var i: usize = 0;
    while (i < space.arbiters.items.len) {
        const ar = space.arbiters.items[i];
        const handler = selectHandler(space, ar.shape_a, ar.shape_b);
        if (matchesBody(ar, body)) {
            if (handler.separate) |sep_func| {
                var temp = space.arbiters.items[i];
                sep_func(&temp, space);
            }
            const removed = removeArbiterAt(space, i);
            stashArbiter(space, removed);
            continue;
        }
        i += 1;
    }

    space.stale_arbiter_keys.clearRetainingCapacity();

    var it = space.arbiter_cache.iterator();
    while (it.next()) |entry| {
        if (matchesBody(entry.value_ptr.value, body)) {
            space.stale_arbiter_keys.append(space.allocator, entry.key_ptr.*) catch {};
        }
    }

    for (space.stale_arbiter_keys.items) |key| {
        if (space.arbiter_cache.remove(key)) |cached| {
            const handler = selectHandler(space, cached.value.shape_a, cached.value.shape_b);
            if (handler.separate) |sep_func| {
                var temp = cached.value;
                sep_func(&temp, space);
            }
            stashArbiter(space, cached.value);
        }
    }
}

const BroadPhaseContext = struct {
    space: *cpSpace,
    primary: *shape_base.cpShape,
    static_query: bool,
    mutex: ?*std.Thread.Mutex,
};

fn queryPairs(object: *const anyopaque, _: bb.cpBB, ctx_ptr: ?*anyopaque) void {
    const ctx = @as(*BroadPhaseContext, @ptrCast(ctx_ptr.?));
    const other = @as(*shape_base.cpShape, @ptrCast(object));
    if (other == ctx.primary) return;
    if (!ctx.static_query) {
        if (@intFromPtr(other) <= @intFromPtr(ctx.primary)) return;
    }
    if (ctx.primary.body.sleeping and other.body.sleeping) return;
    if (shape_base.cpShapeFilter.reject(ctx.primary.filter, other.filter)) return;

    if (ctx.mutex) |lock| {
        lock.lock();
        defer lock.unlock();
    }

    const handler = selectHandler(ctx.space, ctx.primary, other);
    const result = collision.collide(&ctx.space.collision_cache, ctx.primary, other);
    if (result.contactCount() == 0) return;

    activatePair(ctx.space, ctx.primary, other);

    const cached = fetchArbiter(ctx.space, ctx.primary, other) orelse return;
    cached.value.syncContacts(result);

    if (handler.begin) |begin_func| {
        if (!begin_func(&cached.value, ctx.space)) return;
    }
    if (handler.preSolve) |pre_func| {
        if (!pre_func(&cached.value, ctx.space)) return;
    }

    ctx.space.arbiters.append(cached.value) catch {};
}
fn asCircle(shape: *shape_base.cpShape) *circle.cpCircleShape {
    return @as(*circle.cpCircleShape, @ptrCast(shape));
}

fn asSegment(shape: *shape_base.cpShape) *segment.cpSegmentShape {
    return @as(*segment.cpSegmentShape, @ptrCast(shape));
}

fn asPoly(shape: *shape_base.cpShape) *poly.cpPolyShape {
    return @as(*poly.cpPolyShape, @ptrCast(shape));
}

fn castPayload(comptime T: type, payload: *anyopaque) *T {
    return @as(*T, @ptrCast(payload));
}

pub fn opsForPinJoint(_: *joints.PinJoint) ConstraintOps {
    return .{
        .preStep = pinPreStep,
        .applyCachedImpulse = pinApplyCached,
        .applyImpulse = pinApplyImpulse,
    };
}

fn pinPreStep(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.preStep(dt);
}

fn pinApplyCached(payload: *anyopaque, dt_coef: types.cpFloat) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.applyCachedImpulse(dt_coef);
}

fn pinApplyImpulse(payload: *anyopaque) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.applyImpulse();
}

pub fn opsForSlideJoint(_: *joints.SlideJoint) ConstraintOps {
    return .{ .postStep = slidePostStep };
}

fn slidePostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.SlideJoint, payload);
    joint.solvePositions();
}

pub fn opsForPivotJoint(_: *joints.PivotJoint) ConstraintOps {
    return .{ .postStep = pivotPostStep };
}

fn pivotPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.PivotJoint, payload);
    joint.solvePositions();
}

pub fn opsForGrooveJoint(_: *joints.GrooveJoint) ConstraintOps {
    return .{ .postStep = groovePostStep };
}

fn groovePostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.GrooveJoint, payload);
    joint.solvePositions();
}

pub fn opsForDampedSpring(_: *joints.DampedSpring) ConstraintOps {
    return .{ .preStep = dampedSpringPreStep };
}

fn dampedSpringPreStep(payload: *anyopaque, _: types.cpFloat) void {
    const joint = castPayload(joints.DampedSpring, payload);
    joint.applyForces();
}

pub fn opsForDampedRotarySpring(_: *joints.DampedRotarySpring) ConstraintOps {
    return .{ .preStep = dampedRotaryPreStep };
}

fn dampedRotaryPreStep(payload: *anyopaque, _: types.cpFloat) void {
    const joint = castPayload(joints.DampedRotarySpring, payload);
    joint.applyTorques();
}

pub fn opsForSimpleMotor(_: *joints.SimpleMotor) ConstraintOps {
    return .{ .applyImpulse = simpleMotorIterate };
}

fn simpleMotorIterate(payload: *anyopaque) void {
    const joint = castPayload(joints.SimpleMotor, payload);
    joint.drive();
}

pub fn opsForGearJoint(_: *joints.GearJoint) ConstraintOps {
    return .{ .applyImpulse = gearJointIterate, .postStep = gearJointIterate };
}

fn gearJointIterate(payload: *anyopaque) void {
    const joint = castPayload(joints.GearJoint, payload);
    joint.matchAngularVelocity();
    joint.solveAngles();
}

pub fn opsForRatchetJoint(_: *joints.RatchetJoint) ConstraintOps {
    return .{ .postStep = ratchetPostStep };
}

fn ratchetPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.RatchetJoint, payload);
    joint.applyLimits();
}

pub fn opsForRotaryLimitJoint(_: *joints.RotaryLimitJoint) ConstraintOps {
    return .{ .postStep = rotaryLimitPostStep };
}

fn rotaryLimitPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.RotaryLimitJoint, payload);
    joint.clampAngles();
}

pub fn testSpaceIntegration() !void {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();
    space.gravity = vect.cpv(0.0, -9.8);
    space.damping = 0.9;

    var body = body_mod.cpBody.init(1.0, 1.0);
    try space.addBody(&body);

    var shape = circle.cpCircleShape.init(&body, 0.5, vect.cpvzero);
    try space.addShape(&shape.base);

    space.step(1.0);

    try std.testing.expectApproxEqAbs(-8.82, body.v.y, 1e-6);
    try std.testing.expect(body.p.y < 0.0);
}

pub fn testSpaceCollisionResolution() !void {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(1.0, 1.0);
    body_a.setPosition(vect.cpv(-0.5, 0.0));
    body_b.setPosition(vect.cpv(0.5, 0.0));
    body_a.setVelocity(vect.cpv(2.0, 0.0));
    body_b.setVelocity(vect.cpv(-2.0, 0.0));

    var shape_a = circle.cpCircleShape.init(&body_a, 0.5, vect.cpvzero);
    var shape_b = circle.cpCircleShape.init(&body_b, 0.5, vect.cpvzero);

    try space.addBody(&body_a);
    try space.addBody(&body_b);
    try space.addShape(&shape_a.base);
    try space.addShape(&shape_b.base);

    space.step(0.5);

    try std.testing.expect(body_a.p.x < 0.0);
    try std.testing.expect(body_b.p.x > 0.0);
    try std.testing.expect(body_a.v.x <= 0.0);
    try std.testing.expect(body_b.v.x >= 0.0);
}

pub fn testSpaceConstraintPipeline() !void {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(1.0, 1.0);
    body_a.setPosition(vect.cpvzero);
    body_b.setPosition(vect.cpv(5.0, 0.0));

    var joint = joints.PinJoint.init(&body_a, &body_b, vect.cpvzero, vect.cpvzero, 2.0);

    try space.addBody(&body_a);
    try space.addBody(&body_b);
    try space.addConstraint(&joint.base, opsForPinJoint(&joint), null);

    space.step(0.1);

    try std.testing.expect(vect.cpvdist(body_a.p, body_b.p) <= 2.0 + 1e-3);
}

pub fn testSpaceQueries() !void {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(1.0, 1.0));
    var circle_shape = circle.cpCircleShape.init(&body, 1.0, vect.cpvzero);

    try space.addBody(&body);
    try space.addShape(&circle_shape.base);

    var hits: usize = 0;
    const QueryCounter = struct {
        pub var counter: *usize = undefined;
        pub fn cb(_: *shape_base.cpShape, _: vect.cpVect, _: types.cpFloat) void {
            counter.* += 1;
        }
    };
    QueryCounter.counter = &hits;
    space.pointQuery(vect.cpv(1.0, 1.0), shape_base.cpShapeFilter.all(), QueryCounter.cb);

    try std.testing.expectEqual(@as(usize, 1), hits);
}

pub fn testSpaceSegmentQueryFirst() !void {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpvzero);
    var circle_shape = circle.cpCircleShape.init(&body, 1.0, vect.cpvzero);

    try space.addBody(&body);
    try space.addShape(&circle_shape.base);

    var info: shape_base.cpSegmentQueryInfo = undefined;
    const hit = space.segmentQueryFirst(
        vect.cpv(-2.0, 0.0),
        vect.cpv(2.0, 0.0),
        0.0,
        shape_base.cpShapeFilter.all(),
        &info,
    );

    try std.testing.expect(hit == &circle_shape.base);
    try std.testing.expectApproxEqAbs(-1.0, info.point.x, 1e-6);
    try std.testing.expectApproxEqAbs(0.0, info.point.y, 1e-6);
    try std.testing.expectApproxEqAbs(0.25, info.alpha, 1e-6);
    try std.testing.expectApproxEqAbs(-1.0, info.normal.x, 1e-6);
    try std.testing.expectApproxEqAbs(0.0, info.normal.y, 1e-6);
}

test "space integration applies gravity" {
    try testSpaceIntegration();
}

test "space resolves circle collisions" {
    try testSpaceCollisionResolution();
}

test "space enforces constraint distances" {
    try testSpaceConstraintPipeline();
}

test "space queries find overlapping shapes" {
    try testSpaceQueries();
}

test "space segmentQueryFirst reports nearest hit" {
    try testSpaceSegmentQueryFirst();
}

test "sleeping bodies keep arbiter links for activation" {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();
    space.gravity = vect.cpvzero;
    space.sleep_time_threshold = 0.0;

    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(1.0, 1.0);
    body_a.setPosition(vect.cpvzero);
    body_b.setPosition(vect.cpv(0.5, 0.0));

    var shape_a = circle.cpCircleShape.init(&body_a, 0.5, vect.cpvzero);
    var shape_b = circle.cpCircleShape.init(&body_b, 0.5, vect.cpvzero);

    try space.addBody(&body_a);
    try space.addBody(&body_b);
    try space.addShape(&shape_a.base);
    try space.addShape(&shape_b.base);

    space.resolveCollisions();
    try std.testing.expect(body_a.arbiter_list != null);
    try std.testing.expect(body_b.arbiter_list != null);

    space.processComponents(0.1);
    try std.testing.expect(body_a.sleeping);
    try std.testing.expect(body_b.sleeping);

    space.startBroadPhase();
    space.resolveCollisionsRange(0, space.dynamic_shapes.items.len, null);
    space.finishBroadPhase();

    try std.testing.expect(body_a.arbiter_list != null);
    try std.testing.expect(body_b.arbiter_list != null);
}

test "space reindexes shapes for moved bodies" {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setType(.static);
    body.setPosition(vect.cpvzero);
    var circle_shape = circle.cpCircleShape.init(&body, 1.0, vect.cpvzero);

    try space.addBody(&body);
    try space.addShape(&circle_shape.base);

    var hits: usize = 0;
    const Counter = struct {
        pub var out: *usize = undefined;
        pub fn cb(_: *shape_base.cpShape, _: vect.cpVect, _: types.cpFloat) void {
            out.* += 1;
        }
    };
    Counter.out = &hits;

    space.pointQuery(vect.cpvzero, shape_base.cpShapeFilter.all(), Counter.cb);
    try std.testing.expectEqual(@as(usize, 1), hits);

    body.setPosition(vect.cpv(10.0, 0.0));
    hits = 0;
    space.reindexShapesForBody(&body);

    space.pointQuery(vect.cpvzero, shape_base.cpShapeFilter.all(), Counter.cb);
    try std.testing.expectEqual(@as(usize, 0), hits);

    space.pointQuery(vect.cpv(10.0, 0.0), shape_base.cpShapeFilter.all(), Counter.cb);
    try std.testing.expectEqual(@as(usize, 1), hits);
}

test "space swaps indices to spatial hash" {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body = body_mod.cpBody.init(1.0, 1.0);
    var circle_shape = circle.cpCircleShape.init(&body, 1.0, vect.cpvzero);

    try space.addBody(&body);
    try space.addShape(&circle_shape.base);

    try space.useSpatialHash(25.0, 64);

    try std.testing.expectEqual(IndexKind.space_hash, space.dynamic_index.kind);
    try std.testing.expectEqual(IndexKind.space_hash, space.static_index.kind);
    try std.testing.expectEqual(space.dynamic_shapes.items.len, space.dynamic_index.count());
    try std.testing.expectEqual(space.static_shapes.items.len, space.static_index.count());
}

test "space executes post-step callbacks" {
    var space = cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var called = false;
    const Callback = struct {
        fn run(_: *cpSpace, data: *anyopaque) void {
            const flag = @as(*bool, @ptrCast(data));
            flag.* = true;
        }
    };

    try space.addPostStep(null, Callback.run, &called);
    space.step(0.0);

    try std.testing.expect(called);
    try std.testing.expectEqual(@as(usize, 0), space.post_steps.items.len);
}

comptime {
    std.testing.refAllDecls(@This());
}
