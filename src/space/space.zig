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
const pool = @import("../util/pool.zig");

const ShapePair = struct { a: usize, b: usize };

pub const ConstraintOps = struct {
    preStep: ?fn (*anyopaque, types.cpFloat) void = null,
    applyCachedImpulse: ?fn (*anyopaque, types.cpFloat) void = null,
    applyImpulse: ?fn (*anyopaque, types.cpFloat) void = null,
    postStep: ?fn (*anyopaque) void = null,
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

pub const cpSpace = struct {
    allocator: std.mem.Allocator,
    gravity: vect.cpVect = vect.cpvzero,
    damping: types.cpFloat = 1.0,
    iterations: usize = 10,

    bodies: std.ArrayList(*body_mod.cpBody),
    shapes: std.ArrayList(*shape_base.cpShape),
    constraints: std.ArrayList(ConstraintEntry),
    arbiters: std.ArrayList(*arbiter.cpArbiter),
    post_steps: std.ArrayList(PostStepCallback),
    handler: CollisionHandler = .{},
    arbiter_pool: pool.ObjectPool(arbiter.cpArbiter),
    contact_pool: pool.ObjectPool(arbiter.ContactBuffer),
    scratch: std.heap.ArenaAllocator,
    stamp: usize = 0,
    arbiter_map: std.AutoHashMap(ShapePair, *arbiter.cpArbiter),
    stale_pairs: std.ArrayList(ShapePair),

    pub fn init(allocator: std.mem.Allocator) !cpSpace {
        var space = cpSpace{
            .allocator = allocator,
            .bodies = std.ArrayList(*body_mod.cpBody).init(allocator),
            .shapes = std.ArrayList(*shape_base.cpShape).init(allocator),
            .constraints = std.ArrayList(ConstraintEntry).init(allocator),
            .arbiters = std.ArrayList(*arbiter.cpArbiter).init(allocator),
            .post_steps = std.ArrayList(PostStepCallback).init(allocator),
            .handler = .{},
            .arbiter_pool = undefined,
            .contact_pool = undefined,
            .scratch = std.heap.ArenaAllocator.init(allocator),
            .arbiter_map = std.AutoHashMap(ShapePair, *arbiter.cpArbiter).init(allocator),
            .stale_pairs = std.ArrayList(ShapePair).init(allocator),
        };
        errdefer space.scratch.deinit();
        errdefer space.arbiter_map.deinit();
        errdefer space.stale_pairs.deinit();
        space.arbiter_pool = try pool.ObjectPool(arbiter.cpArbiter).init(allocator, 32);
        errdefer space.arbiter_pool.deinit();
        space.contact_pool = try pool.ObjectPool(arbiter.ContactBuffer).init(allocator, 64);
        return space;
    }

    pub fn deinit(self: *cpSpace) void {
        var it = self.arbiter_map.iterator();
        while (it.next()) |entry| {
            recycleArbiter(self, entry.value_ptr.*);
        }
        self.bodies.deinit();
        self.shapes.deinit();
        self.constraints.deinit();
        self.arbiters.deinit();
        self.post_steps.deinit();
        self.arbiter_pool.deinit();
        self.contact_pool.deinit();
        self.scratch.deinit();
        self.arbiter_map.deinit();
        self.stale_pairs.deinit();
    }

    pub fn addBody(self: *cpSpace, body: *body_mod.cpBody) !void {
        try self.bodies.append(body);
    }

    pub fn removeBody(self: *cpSpace, body: *body_mod.cpBody) void {
        removePtr(*body_mod.cpBody, &self.bodies, body);
    }

    pub fn addShape(self: *cpSpace, shape: *shape_base.cpShape) !void {
        try self.shapes.append(shape);
    }

    pub fn removeShape(self: *cpSpace, shape: *shape_base.cpShape) void {
        removePtr(*shape_base.cpShape, &self.shapes, shape);
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
        const dt_coef: types.cpFloat = if (dt != 0.0) dt else 1.0;
        self.stamp += 1;
        updateVelocities(self, dt);
        runConstraintCallback(self.constraints.items, dt, dt_coef, .preStep);
        runConstraintCallback(self.constraints.items, dt, dt_coef, .applyCachedImpulse);
        updateShapeCaches(self);
        buildArbiters(self, dt, dt_coef);

        var iteration: usize = 0;
        while (iteration < self.iterations) : (iteration += 1) {
            runConstraintCallback(self.constraints.items, dt, dt_coef, .applyImpulse);
            resolveArbiters(self, dt);
        }

        integratePositions(self, dt);
        finalizeArbiters(self);
        runConstraintCallback(self.constraints.items, dt, dt_coef, .postStep);
        runPostSteps(self);
    }

    pub fn pointQuery(self: *const cpSpace, point: vect.cpVect, filter: shape_base.cpShapeFilter, func: fn (*shape_base.cpShape, vect.cpVect, types.cpFloat) void) void {
        for (self.shapes.items) |shape| {
            if (shape_base.cpShapeFilter.reject(shape.filter, filter)) continue;
            const distance = pointDistance(shape, point);
            if (distance <= 0.0) {
                func(shape, point, distance);
            }
        }
    }

    pub fn bbQuery(self: *const cpSpace, bounds: bb.cpBB, filter: shape_base.cpShapeFilter, func: fn (*shape_base.cpShape) void) void {
        for (self.shapes.items) |shape| {
            if (shape_base.cpShapeFilter.reject(shape.filter, filter)) continue;
            if (bb.cpBBIntersects(shape.bbValue(), bounds)) {
                func(shape);
            }
        }
    }

    pub fn shapeQuery(self: *const cpSpace, target: *shape_base.cpShape, func: fn (*shape_base.cpShape, collision.CollisionResult) void) void {
        for (self.shapes.items) |shape| {
            if (shape == target) continue;
            const mark = self.scratch.state();
            defer self.scratch.restore(mark);
            const result = collision.collide(self.scratch.allocator(), target, shape);
            if (result.contactCount() > 0) {
                func(shape, result);
            }
        }
    }
};

fn removePtr(comptime T: type, list: *std.ArrayList(T), target: T) void {
    var i: usize = 0;
    while (i < list.items.len) : (i += 1) {
        if (list.items[i] == target) {
            _ = list.orderedRemove(i);
            break;
        }
    }
}

fn runConstraintCallback(constraints: []ConstraintEntry, dt: types.cpFloat, dt_coef: types.cpFloat, comptime which: enum { preStep, applyCachedImpulse, applyImpulse, postStep }) void {
    for (constraints) |entry| {
        switch (which) {
            .preStep => if (entry.ops.preStep) |fn_ptr| fn_ptr(entry.payload, dt),
            .applyCachedImpulse => if (entry.ops.applyCachedImpulse) |fn_ptr| fn_ptr(entry.payload, dt_coef),
            .applyImpulse => if (entry.ops.applyImpulse) |fn_ptr| fn_ptr(entry.payload, dt),
            .postStep => if (entry.ops.postStep) |fn_ptr| fn_ptr(entry.payload),
        }
    }
}

fn updateVelocities(space: *cpSpace, dt: types.cpFloat) void {
    for (space.bodies.items) |body| {
        body.updateVelocity(space.gravity, space.damping, dt);
    }
}

fn integratePositions(space: *cpSpace, dt: types.cpFloat) void {
    for (space.bodies.items) |body| {
        body.updatePosition(dt);
    }
}

fn updateShapeCaches(space: *cpSpace) void {
    for (space.shapes.items) |shape| {
        switch (shape.shape_type) {
            .circle => asCircle(shape).cacheBB(),
            .segment => asSegment(shape).cacheBB(),
            .poly => asPoly(shape).cacheBB(),
        }
    }
}

fn buildArbiters(space: *cpSpace, dt: types.cpFloat, dt_coef: types.cpFloat) void {
    space.arbiters.clearRetainingCapacity();
    space.scratch.reset(.retain_capacity);

    for (space.shapes.items, 0..) |shape_a, i| {
        var j: usize = i + 1;
        while (j < space.shapes.items.len) : (j += 1) {
            const shape_b = space.shapes.items[j];
            if (shape_base.cpShapeFilter.reject(shape_a.filter, shape_b.filter)) continue;
            const mark = space.scratch.state();
            defer space.scratch.restore(mark);
            const result = collision.collide(space.scratch.allocator(), shape_a, shape_b);
            if (result.contactCount() == 0) continue;

            const pair = makePair(shape_a, shape_b);
            const gop = space.arbiter_map.getOrPut(pair) catch continue;
            var arb_ptr: *arbiter.cpArbiter = undefined;
            if (!gop.found_existing) {
                const buffer = space.contact_pool.acquire() catch continue;
                buffer.* = arbiter.ContactBuffer.init();
                const new_arb = space.arbiter_pool.acquire() catch {
                    space.contact_pool.release(buffer);
                    continue;
                };
                new_arb.* = arbiter.cpArbiter.init(shape_a, shape_b, buffer);
                gop.value_ptr.* = new_arb;
                arb_ptr = new_arb;
            } else {
                arb_ptr = gop.value_ptr.*;
                arb_ptr.updateShapes(shape_a, shape_b);
            }

            arb_ptr.stamp = space.stamp;
            arb_ptr.updateContacts(result);

            if (arb_ptr.state == .ignore) continue;

            if (arb_ptr.state == .first) {
                const allow_begin = if (space.handler.begin) |begin_func| begin_func(arb_ptr, space) else true;
                if (!allow_begin) {
                    arb_ptr.state = .ignore;
                    continue;
                }
                arb_ptr.state = .normal;
            }

            var allowed = true;
            if (space.handler.preSolve) |pre_func| {
                allowed = pre_func(arb_ptr, space);
            }
            if (!allowed) continue;

            arb_ptr.preStep(dt);
            arb_ptr.applyCachedImpulse(dt_coef);
            space.arbiters.append(arb_ptr) catch {};
        }
    }

    space.stale_pairs.clearRetainingCapacity();
    var it = space.arbiter_map.iterator();
    while (it.next()) |entry| {
        if (entry.value_ptr.*.stamp != space.stamp) {
            space.stale_pairs.append(entry.key_ptr.*) catch {};
        }
    }
}

fn resolveArbiters(space: *cpSpace, dt: types.cpFloat) void {
    for (space.arbiters.items) |arb_ptr| {
        arb_ptr.applyImpulse(dt);
    }
}

fn finalizeArbiters(space: *cpSpace) void {
    if (space.handler.postSolve) |post_func| {
        for (space.arbiters.items) |arb_ptr| {
            post_func(arb_ptr, space);
        }
    }

    if (space.stale_pairs.items.len == 0) return;
    for (space.stale_pairs.items) |pair| {
        if (space.arbiter_map.fetchRemove(pair)) |entry| {
            const arb_ptr = entry.value;
            if (space.handler.separate) |sep_func| {
                sep_func(arb_ptr, space);
            }
            recycleArbiter(space, arb_ptr);
        }
    }
    space.stale_pairs.clearRetainingCapacity();
}

fn runPostSteps(space: *cpSpace) void {
    for (space.post_steps.items) |callback| {
        callback.func(space, callback.data);
    }
    space.post_steps.clearRetainingCapacity();
}

fn recycleArbiter(space: *cpSpace, arb: *arbiter.cpArbiter) void {
    space.contact_pool.release(arb.buffer);
    space.arbiter_pool.release(arb);
}

fn makePair(a: *shape_base.cpShape, b: *shape_base.cpShape) ShapePair {
    return .{ .a = @intFromPtr(a), .b = @intFromPtr(b) };
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

fn pointDistance(shape: *const shape_base.cpShape, point: vect.cpVect) types.cpFloat {
    return switch (shape.shape_type) {
        .circle => distanceToCircle(asCircle(shape), point),
        .segment => distanceToSegment(asSegment(shape), point),
        .poly => distanceToPoly(asPoly(shape), point),
    };
}

fn distanceToCircle(shape: *const circle.cpCircleShape, point: vect.cpVect) types.cpFloat {
    const center = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.offset, shape.base.body.rotationVector()));
    return vect.cpvlength(vect.cpvsub(point, center)) - shape.radius;
}

fn distanceToSegment(shape: *const segment.cpSegmentShape, point: vect.cpVect) types.cpFloat {
    const rot = shape.base.body.rotationVector();
    const a = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.a, rot));
    const b = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.b, rot));
    const nearest = closestPoint(point, a, b);
    return vect.cpvlength(vect.cpvsub(point, nearest)) - shape.radius;
}

fn closestPoint(p: vect.cpVect, a: vect.cpVect, b_: vect.cpVect) vect.cpVect {
    const ab = vect.cpvsub(b_, a);
    const denom = vect.cpvdot(ab, ab);
    if (denom == 0.0) return a;
    const t = types.cpfclamp(vect.cpvdot(vect.cpvsub(p, a), ab) / denom, 0.0, 1.0);
    return vect.cpvadd(a, vect.cpvmult(ab, t));
}

fn distanceToPoly(shape: *const poly.cpPolyShape, point: vect.cpVect) types.cpFloat {
    var inside = true;
    var min_dist = types.CP_INFINITY;
    const rot = shape.base.body.rotationVector();
    for (shape.vertices, 0..) |vertex, i| {
        const world_a = vect.cpvadd(shape.base.body.p, vect.cpvrotate(vertex, rot));
        const world_b = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.vertices[(i + 1) % shape.vertices.len], rot));
        const edge = vect.cpvsub(world_b, world_a);
        const normal = vect.cpvperp(edge);
        const dist = vect.cpvdot(normal, vect.cpvsub(point, world_a));
        if (dist > 0.0) inside = false;
        const projected = closestPoint(point, world_a, world_b);
        min_dist = types.cpfmin(min_dist, vect.cpvlength(vect.cpvsub(point, projected)));
    }
    return if (inside) -min_dist else min_dist;
}

pub fn opsForPinJoint(_: *joints.PinJoint) ConstraintOps {
    return .{ .preStep = pinPreStep, .applyCachedImpulse = pinApplyCachedImpulse, .applyImpulse = pinApplyImpulse, .postStep = pinPostStep };
}

fn pinPreStep(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.preStep(dt);
}

fn pinApplyCachedImpulse(payload: *anyopaque, dt_coef: types.cpFloat) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.applyCachedImpulse(dt_coef);
}

fn pinApplyImpulse(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.applyImpulse(dt);
}

fn pinPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.PinJoint, payload);
    joint.solvePositions();
}

pub fn opsForSlideJoint(_: *joints.SlideJoint) ConstraintOps {
    return .{ .preStep = slidePreStep, .applyCachedImpulse = slideApplyCachedImpulse, .applyImpulse = slideApplyImpulse, .postStep = slidePostStep };
}

fn slidePreStep(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.SlideJoint, payload);
    joint.preStep(dt);
}

fn slideApplyCachedImpulse(payload: *anyopaque, dt_coef: types.cpFloat) void {
    const joint = castPayload(joints.SlideJoint, payload);
    joint.applyCachedImpulse(dt_coef);
}

fn slideApplyImpulse(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.SlideJoint, payload);
    joint.applyImpulse(dt);
}

fn slidePostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.SlideJoint, payload);
    joint.solvePositions();
}

pub fn opsForPivotJoint(_: *joints.PivotJoint) ConstraintOps {
    return .{
        .preStep = pivotPreStep,
        .applyCachedImpulse = pivotApplyCachedImpulse,
        .applyImpulse = pivotApplyImpulse,
        .postStep = pivotPostStep,
    };
}

fn pivotPreStep(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.PivotJoint, payload);
    joint.preStep(dt);
}

fn pivotApplyCachedImpulse(payload: *anyopaque, dt_coef: types.cpFloat) void {
    const joint = castPayload(joints.PivotJoint, payload);
    joint.applyCachedImpulse(dt_coef);
}

fn pivotApplyImpulse(payload: *anyopaque, dt: types.cpFloat) void {
    const joint = castPayload(joints.PivotJoint, payload);
    joint.applyImpulse(dt);
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

fn simpleMotorIterate(payload: *anyopaque, _: types.cpFloat) void {
    const joint = castPayload(joints.SimpleMotor, payload);
    joint.drive();
}

pub fn opsForGearJoint(_: *joints.GearJoint) ConstraintOps {
    return .{ .applyImpulse = gearJointApplyImpulse, .postStep = gearJointPostStep };
}

fn gearJointApplyImpulse(payload: *anyopaque, _: types.cpFloat) void {
    const joint = castPayload(joints.GearJoint, payload);
    joint.matchAngularVelocity();
}

fn gearJointPostStep(payload: *anyopaque) void {
    const joint = castPayload(joints.GearJoint, payload);
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
    var space = try cpSpace.init(std.testing.allocator);
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
    var space = try cpSpace.init(std.testing.allocator);
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
    var space = try cpSpace.init(std.testing.allocator);
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
    var space = try cpSpace.init(std.testing.allocator);
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
