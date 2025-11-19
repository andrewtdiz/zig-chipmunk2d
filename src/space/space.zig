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

fn shapeBounds(ptr: *const anyopaque, _: ?*const anyopaque) bb.cpBB {
    const shape = @as(*const shape_base.cpShape, @ptrCast(ptr));
    return shape.bbValue();
}

pub const cpSpace = struct {
    allocator: std.mem.Allocator,
    gravity: vect.cpVect = vect.cpvzero,
    damping: types.cpFloat = 1.0,
    iterations: usize = 10,

    bodies: std.ArrayList(*body_mod.cpBody),
    shapes: std.ArrayList(*shape_base.cpShape),
    dynamic_shapes: std.ArrayList(*shape_base.cpShape),
    static_shapes: std.ArrayList(*shape_base.cpShape),
    constraints: std.ArrayList(ConstraintEntry),
    arbiters: std.ArrayList(arbiter.cpArbiter),
    post_steps: std.ArrayList(PostStepCallback),
    handler: CollisionHandler = .{},
    collision_cache: collision.CollisionCache,
    dynamic_index: ManagedIndex,
    static_index: ManagedIndex,

    pub fn init(allocator: std.mem.Allocator) cpSpace {
        return .{
            .allocator = allocator,
            .bodies = std.ArrayList(*body_mod.cpBody).init(allocator),
            .shapes = std.ArrayList(*shape_base.cpShape).init(allocator),
            .dynamic_shapes = std.ArrayList(*shape_base.cpShape).init(allocator),
            .static_shapes = std.ArrayList(*shape_base.cpShape).init(allocator),
            .constraints = std.ArrayList(ConstraintEntry).init(allocator),
            .arbiters = std.ArrayList(arbiter.cpArbiter).init(allocator),
            .post_steps = std.ArrayList(PostStepCallback).init(allocator),
            .collision_cache = collision.CollisionCache.init(allocator),
            .dynamic_index = ManagedIndex.init(allocator, .bb_tree, shapeBounds, null, null),
            .static_index = ManagedIndex.init(allocator, .bb_tree, shapeBounds, null, null),
        };
    }

    pub fn deinit(self: *cpSpace) void {
        self.bodies.deinit();
        self.shapes.deinit();
        self.dynamic_shapes.deinit();
        self.static_shapes.deinit();
        self.constraints.deinit();
        self.arbiters.deinit();
        self.post_steps.deinit();
        self.collision_cache.deinit();
        self.dynamic_index.deinit();
        self.static_index.deinit();
    }

    pub fn addBody(self: *cpSpace, body: *body_mod.cpBody) !void {
        try self.bodies.append(body);
    }

    pub fn removeBody(self: *cpSpace, body: *body_mod.cpBody) void {
        removePtr(*body_mod.cpBody, &self.bodies, body);
    }

    pub fn addShape(self: *cpSpace, shape: *shape_base.cpShape) !void {
        try self.shapes.append(shape);
        cacheShapeInternal(shape);
        const ptr = @as(*const anyopaque, @ptrCast(shape));
        switch (shape.body.body_type) {
            .static => {
                try self.static_shapes.append(shape);
                try self.static_index.insert(ptr);
            },
            else => {
                try self.dynamic_shapes.append(shape);
                try self.dynamic_index.insert(ptr);
            },
        }
    }

    pub fn removeShape(self: *cpSpace, shape: *shape_base.cpShape) void {
        removePtr(*shape_base.cpShape, &self.shapes, shape);
        const ptr = @as(*const anyopaque, @ptrCast(shape));
        self.collision_cache.removeShape(shape);
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
        try self.constraints.append(.{ .constraint = constraint, .payload = stored_payload, .ops = ops });
    }

    pub fn removeConstraint(self: *cpSpace, constraint: *constraint_base.cpConstraint) void {
        var i: usize = 0;
        while (i < self.constraints.items.len) : (i += 1) {
            if (self.constraints.items[i].constraint == constraint) {
                _ = self.constraints.orderedRemove(i);
                break;
            }
        }
    }

    pub fn addPostStep(self: *cpSpace, key: ?*const anyopaque, func: fn (*cpSpace, *anyopaque) void, data: *anyopaque) !void {
        try self.post_steps.append(.{ .key = key, .func = func, .data = data });
    }

    pub fn setCollisionHandler(self: *cpSpace, handler: CollisionHandler) void {
        self.handler = handler;
    }

    pub fn step(self: *cpSpace, dt: types.cpFloat) void {
        Stepper.step(self, dt);
    }

    pub fn pointQuery(self: *const cpSpace, point: vect.cpVect, filter: shape_base.cpShapeFilter, func: fn (*shape_base.cpShape, vect.cpVect, types.cpFloat) void) void {
        QueryAPI.pointQuery(self, point, filter, func);
    }

    pub fn bbQuery(self: *const cpSpace, bounds: bb.cpBB, filter: shape_base.cpShapeFilter, func: fn (*shape_base.cpShape) void) void {
        QueryAPI.bbQuery(self, bounds, filter, func);
    }

    pub fn segmentQuery(self: *const cpSpace, start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat, filter: shape_base.cpShapeFilter, func: fn (*shape_base.cpShape, vect.cpVect, vect.cpVect, types.cpFloat) void) void {
        QueryAPI.segmentQuery(self, start, end, radius, filter, func);
    }

    pub fn shapeQuery(self: *const cpSpace, target: *shape_base.cpShape, func: fn (*shape_base.cpShape, collision.CollisionResult) void) void {
        QueryAPI.shapeQuery(self, target, func);
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
        for (self.shapes.items) |shape| {
            if (shape.body == body) {
                self.reindexShape(shape);
            }
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
    for (space.bodies.items) |body| {
        body.updateVelocity(space.gravity, space.damping, dt);
    }
}

pub fn integratePositions(space: *cpSpace, dt: types.cpFloat) void {
    for (space.bodies.items) |body| {
        body.updatePosition(dt);
    }
}

pub fn updateShapeCaches(space: *cpSpace) void {
    for (space.shapes.items) |shape| {
        cacheShapeInternal(shape);
    }
}

pub fn resolveCollisions(space: *cpSpace) void {
    space.arbiters.clearRetainingCapacity();
    for (space.shapes.items, 0..) |shape_a, i| {
        var j: usize = i + 1;
        while (j < space.shapes.items.len) : (j += 1) {
            const shape_b = space.shapes.items[j];
            if (shape_base.cpShapeFilter.reject(shape_a.filter, shape_b.filter)) continue;
            const result = collision.collide(&space.collision_cache, shape_a, shape_b);
            if (result.contactCount() == 0) continue;

            var new_arb = arbiter.cpArbiter.init(shape_a, shape_b);
            new_arb.collision_id = result.id;
            for (result.contacts.constSlice()) |contact| {
                new_arb.addContact(contact);
            }

            if (space.handler.begin) |begin_func| {
                if (!begin_func(&new_arb, space)) continue;
            }
            if (space.handler.preSolve) |pre_func| {
                if (!pre_func(&new_arb, space)) continue;
            }

            space.arbiters.append(new_arb) catch {};
        }
    }
}

pub fn resolveArbiters(space: *cpSpace) void {
    resolveArbitersRange(space, 0, space.arbiters.items.len);
}

pub fn resolveArbitersRange(space: *cpSpace, start: usize, end: usize) void {
    var idx = start;
    while (idx < end) : (idx += 1) {
        var arb_ptr = &space.arbiters.items[idx];
        const shape_a = arb_ptr.shape_a;
        const shape_b = arb_ptr.shape_b;
        const body_a = shape_a.body;
        const body_b = shape_b.body;

        for (arb_ptr.contacts.constSlice()) |contact| {
            if (contact.distance >= 0.0) continue;
            const total_inv = body_a.m_inv + body_b.m_inv;
            if (total_inv == 0.0) continue;

            const penetration = -contact.distance;
            const correction = vect.cpvmult(contact.normal, penetration);
            body_a.p = vect.cpvsub(body_a.p, vect.cpvmult(correction, body_a.m_inv / total_inv));
            body_b.p = vect.cpvadd(body_b.p, vect.cpvmult(correction, body_b.m_inv / total_inv));

            const relative = vect.cpvsub(body_b.v, body_a.v);
            const vel_normal = vect.cpvdot(relative, contact.normal);
            if (vel_normal > 0.0) continue;
            const elasticity = types.cpfmax(shape_a.elasticity, shape_b.elasticity);
            const impulse = -(1.0 + elasticity) * vel_normal / total_inv;
            const impulse_vec = vect.cpvmult(contact.normal, impulse);
            body_a.v = vect.cpvsub(body_a.v, vect.cpvmult(impulse_vec, body_a.m_inv));
            body_b.v = vect.cpvadd(body_b.v, vect.cpvmult(impulse_vec, body_b.m_inv));
        }

        if (space.handler.separate) |sep_func| {
            sep_func(arb_ptr, space);
        }
    }
}

pub fn postSolveArbiters(space: *cpSpace) void {
    if (space.handler.postSolve) |post_func| {
        for (space.arbiters.items) |*arb_ref| {
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

fn cacheShapeInternal(shape: *shape_base.cpShape) void {
    switch (shape.shape_type) {
        .circle => asCircle(shape).cacheBB(),
        .segment => asSegment(shape).cacheBB(),
        .poly => asPoly(shape).cacheBB(),
    }
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
    return .{ .postStep = pinPostStep };
}

fn pinPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.solvePositions();
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
