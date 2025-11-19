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

pub const cpSpace = struct {
    allocator: std.mem.Allocator,
    gravity: vect.cpVect = vect.cpvzero,
    damping: types.cpFloat = 1.0,
    iterations: usize = 10,

    bodies: std.ArrayList(*body_mod.cpBody),
    shapes: std.ArrayList(*shape_base.cpShape),
    constraints: std.ArrayList(ConstraintEntry),
    arbiters: std.ArrayList(arbiter.cpArbiter),
    post_steps: std.ArrayList(PostStepCallback),
    handler: CollisionHandler = .{},

    pub fn init(allocator: std.mem.Allocator) cpSpace {
        return .{
            .allocator = allocator,
            .bodies = std.ArrayList(*body_mod.cpBody).init(allocator),
            .shapes = std.ArrayList(*shape_base.cpShape).init(allocator),
            .constraints = std.ArrayList(ConstraintEntry).init(allocator),
            .arbiters = std.ArrayList(arbiter.cpArbiter).init(allocator),
            .post_steps = std.ArrayList(PostStepCallback).init(allocator),
        };
    }

    pub fn deinit(self: *cpSpace) void {
        self.bodies.deinit();
        self.shapes.deinit();
        self.constraints.deinit();
        self.arbiters.deinit();
        self.post_steps.deinit();
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
        updateVelocities(self, dt);
        runConstraintCallback(self.constraints.items, dt, dt_coef, .preStep);
        runConstraintCallback(self.constraints.items, dt, dt_coef, .applyCachedImpulse);
        updateShapeCaches(self);
        resolveCollisions(self);

        var iteration: usize = 0;
        while (iteration < self.iterations) : (iteration += 1) {
            runConstraintCallback(self.constraints.items, dt, dt_coef, .applyImpulse);
            resolveArbiters(self);
        }

        integratePositions(self, dt);
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
            const result = collision.collide(target, shape);
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
        switch (shape.shape_type) {
            .circle => asCircle(shape).cacheBB(),
            .segment => asSegment(shape).cacheBB(),
            .poly => asPoly(shape).cacheBB(),
        }
    }
}

pub fn resolveCollisions(space: *cpSpace) void {
    space.arbiters.clearRetainingCapacity();
    for (space.shapes.items, 0..) |shape_a, i| {
        var j: usize = i + 1;
        while (j < space.shapes.items.len) : (j += 1) {
            const shape_b = space.shapes.items[j];
            if (shape_base.cpShapeFilter.reject(shape_a.filter, shape_b.filter)) continue;
            const result = collision.collide(shape_a, shape_b);
            if (result.contactCount() == 0) continue;

            var new_arb = arbiter.cpArbiter.init(shape_a, shape_b);
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
            if (space.handler.postSolve) |post_func| {
                post_func(&new_arb, space);
            }
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

pub fn runPostSteps(space: *cpSpace) void {
    for (space.post_steps.items) |callback| {
        callback.func(space, callback.data);
    }
    space.post_steps.clearRetainingCapacity();
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
