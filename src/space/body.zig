const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const shape_base = @import("../shape/shape_base.zig");
const constraint_base = @import("../constraint/constraint_base.zig");
const arbiter_mod = @import("../collision/arbiter.zig");

pub const cpBodyType = enum { dynamic, kinematic, static };

pub const cpBody = struct {
    body_type: cpBodyType = .dynamic,
    m: types.cpFloat = 1.0,
    m_inv: types.cpFloat = 1.0,
    i: types.cpFloat = 1.0,
    i_inv: types.cpFloat = 1.0,
    p: vect.cpVect = vect.cpvzero,
    v: vect.cpVect = vect.cpvzero,
    f: vect.cpVect = vect.cpvzero,
    a: types.cpFloat = 0.0,
    w: types.cpFloat = 0.0,
    t: types.cpFloat = 0.0,
    v_bias: vect.cpVect = vect.cpvzero,
    w_bias: types.cpFloat = 0.0,
    sleeping: bool = false,
    idle_stamp: u64 = 0,
    idle_time: types.cpFloat = 0.0,
    user_data: types.cpDataPointer = null,
    shape_list: ?*shape_base.cpShape = null,
    constraint_list: ?*constraint_base.cpConstraint = null,
    arbiter_list: ?*arbiter_mod.cpArbiter = null,

    pub fn init(mass: types.cpFloat, moment: types.cpFloat) cpBody {
        var body = cpBody{};
        body.setMass(mass);
        body.setMoment(moment);
        return body;
    }

    pub fn setMass(self: *cpBody, mass: types.cpFloat) void {
        self.m = mass;
        self.m_inv = if (mass == 0.0) 0.0 else 1.0 / mass;
    }

    pub fn setMoment(self: *cpBody, moment: types.cpFloat) void {
        self.i = moment;
        self.i_inv = if (moment == 0.0) 0.0 else 1.0 / moment;
    }

    pub fn setType(self: *cpBody, body_type: cpBodyType) void {
        self.body_type = body_type;
        switch (body_type) {
            .dynamic => {
                self.m_inv = if (self.m == 0.0) 0.0 else 1.0 / self.m;
                self.i_inv = if (self.i == 0.0) 0.0 else 1.0 / self.i;
            },
            .kinematic, .static => {
                self.m_inv = 0.0;
                self.i_inv = 0.0;
                self.v = vect.cpvzero;
                self.w = 0.0;
                self.f = vect.cpvzero;
                self.t = 0.0;
                self.v_bias = vect.cpvzero;
                self.w_bias = 0.0;
            },
        }
    }

    pub fn setPosition(self: *cpBody, position: vect.cpVect) void {
        self.p = position;
    }

    pub fn setVelocity(self: *cpBody, velocity: vect.cpVect) void {
        self.v = velocity;
    }

    pub fn rotationVector(self: cpBody) vect.cpVect {
        return vect.cpvforangle(self.a);
    }

    pub fn localToWorld(self: *const cpBody, point: vect.cpVect) vect.cpVect {
        const rot = self.rotationVector();
        return vect.cpvadd(self.p, vect.cpvrotate(point, rot));
    }

    pub fn worldToLocal(self: *const cpBody, point: vect.cpVect) vect.cpVect {
        const rot = self.rotationVector();
        return vect.cpvunrotate(vect.cpvsub(point, self.p), rot);
    }

    pub fn attachShape(self: *cpBody, shape: *shape_base.cpShape) void {
        shape.next = self.shape_list;
        self.shape_list = shape;
    }

    pub fn detachShape(self: *cpBody, shape: *shape_base.cpShape) void {
        var cursor = &self.shape_list;
        while (cursor.*) |node| {
            if (node == shape) {
                cursor.* = node.next;
                shape.next = null;
                return;
            }
            cursor = &node.next;
        }
    }

    pub fn attachConstraint(self: *cpBody, constraint: *constraint_base.cpConstraint) void {
        if (constraint.a != self and constraint.b != self) return;
        const next_ptr = constraint.nextPtrForBody(self);
        next_ptr.* = self.constraint_list;
        self.constraint_list = constraint;
    }

    pub fn detachConstraint(self: *cpBody, constraint: *constraint_base.cpConstraint) void {
        var cursor = &self.constraint_list;
        while (cursor.*) |node| {
            if (node == constraint) {
                cursor.* = node.nextForBody(self);
                node.setNextForBody(self, null);
                return;
            }
            cursor = node.nextPtrForBody(self);
        }
    }

    pub fn attachArbiter(self: *cpBody, arb: *arbiter_mod.cpArbiter) void {
        if (!arb.involves(self)) return;
        arb.setNextForBody(self, self.arbiter_list);
        self.arbiter_list = arb;
    }

    pub fn detachArbiter(self: *cpBody, arb: *arbiter_mod.cpArbiter) void {
        var cursor = &self.arbiter_list;
        while (cursor.*) |node| {
            if (node == arb) {
                cursor.* = node.nextForBody(self);
                node.setNextForBody(self, null);
                return;
            }
            cursor = node.nextPtrForBody(self);
        }
    }

    pub fn applyForceAtWorldPoint(self: *cpBody, force: vect.cpVect, point: vect.cpVect) void {
        self.f = vect.cpvadd(self.f, force);
        const r = vect.cpvsub(point, self.p);
        self.t += vect.cpvcross(r, force);
    }

    pub fn applyImpulseAtWorldPoint(self: *cpBody, impulse: vect.cpVect, point: vect.cpVect) void {
        self.v = vect.cpvadd(self.v, vect.cpvmult(impulse, self.m_inv));
        const r = vect.cpvsub(point, self.p);
        self.w += self.i_inv * vect.cpvcross(r, impulse);
    }

    pub fn kineticEnergy(self: cpBody) types.cpFloat {
        const linear = vect.cpvdot(self.v, self.v) * self.m * 0.5;
        const angular = self.w * self.w * self.i * 0.5;
        return linear + angular;
    }

    pub fn updateVelocity(self: *cpBody, gravity: vect.cpVect, damping: types.cpFloat, dt: types.cpFloat) void {
        if (self.body_type == .static) return;

        const dv = vect.cpvadd(gravity, vect.cpvmult(self.f, self.m_inv));
        self.v = vect.cpvmult(vect.cpvadd(self.v, vect.cpvmult(dv, dt)), damping);
        self.w = (self.w + self.t * self.i_inv * dt) * damping;

        self.f = vect.cpvzero;
        self.t = 0.0;
    }

    pub fn updatePosition(self: *cpBody, dt: types.cpFloat) void {
        if (self.body_type == .static) return;

        const delta = vect.cpvadd(self.v, self.v_bias);
        self.p = vect.cpvadd(self.p, vect.cpvmult(delta, dt));
        self.a += (self.w + self.w_bias) * dt;

        self.v_bias = vect.cpvzero;
        self.w_bias = 0.0;
    }
};

pub fn cpBodyLocalToWorld(body: *const cpBody, point: vect.cpVect) vect.cpVect {
    return body.localToWorld(point);
}

pub fn cpBodyWorldToLocal(body: *const cpBody, point: vect.cpVect) vect.cpVect {
    return body.worldToLocal(point);
}

pub fn cpMomentForCircle(mass: types.cpFloat, inner_radius: types.cpFloat, outer_radius: types.cpFloat, offset: vect.cpVect) types.cpFloat {
    const r1_sq = inner_radius * inner_radius;
    const r2_sq = outer_radius * outer_radius;
    return mass * (0.5 * (r1_sq + r2_sq) + vect.cpvlengthsq(offset));
}

pub fn cpMomentForSegment(mass: types.cpFloat, a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) types.cpFloat {
    const offset = vect.cpvlerp(a, b, 0.5);
    const length = vect.cpvdist(a, b) + 2.0 * radius;
    return mass * ((length * length + 4.0 * radius * radius) / 12.0 + vect.cpvlengthsq(offset));
}

pub fn cpMomentForBox(mass: types.cpFloat, width: types.cpFloat, height: types.cpFloat) types.cpFloat {
    return mass * (width * width + height * height) / 12.0;
}

pub fn cpMomentForBox2(mass: types.cpFloat, bounds: @import("../core/bb.zig").cpBB) types.cpFloat {
    const width = bounds.r - bounds.l;
    const height = bounds.t - bounds.b;
    const offset = vect.cpvmult(vect.cpv(bounds.l + bounds.r, bounds.b + bounds.t), 0.5);
    return cpMomentForBox(mass, width, height) + mass * vect.cpvlengthsq(offset);
}

test "cpBody integration" {
    var body = cpBody.init(2.0, 4.0);
    body.setPosition(vect.cpv(0.0, 0.0));
    body.applyForceAtWorldPoint(vect.cpv(4.0, 0.0), vect.cpvzero);
    body.updateVelocity(vect.cpv(0.0, -9.8), 0.9, 1.0);
    try std.testing.expectApproxEqAbs(1.8, body.v.x, 1e-6);
    try std.testing.expectApproxEqAbs(-4.41, body.v.y, 1e-6);

    body.updatePosition(1.0);
    try std.testing.expectApproxEqAbs(1.8, body.p.x, 1e-6);
    try std.testing.expectApproxEqAbs(-4.41, body.p.y, 1e-6);
}

test "cpBody transforms between local and world space" {
    var body = cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(2.0, 3.0));
    body.a = types.CP_PI / 2.0;

    const local = vect.cpv(1.0, -1.0);
    const world = body.localToWorld(local);
    try std.testing.expectApproxEqAbs(3.0, world.x, 1e-6);
    try std.testing.expectApproxEqAbs(4.0, world.y, 1e-6);

    const back = body.worldToLocal(world);
    try std.testing.expectApproxEqAbs(local.x, back.x, 1e-6);
    try std.testing.expectApproxEqAbs(local.y, back.y, 1e-6);

    const alias = cpBodyLocalToWorld(&body, local);
    try std.testing.expectApproxEqAbs(world.x, alias.x, 1e-6);
    try std.testing.expectApproxEqAbs(world.y, alias.y, 1e-6);
}

test "moment helpers" {
    const circle_moment = cpMomentForCircle(2.0, 0.0, 3.0, vect.cpvzero);
    try std.testing.expectApproxEqAbs(9.0, circle_moment, 1e-6);

    const segment_moment = cpMomentForSegment(1.0, vect.cpv(-1.0, 0.0), vect.cpv(1.0, 0.0), 0.5);
    try std.testing.expect(segment_moment > 0.0);

    const box_moment = cpMomentForBox(1.5, 2.0, 4.0);
    try std.testing.expectApproxEqAbs(2.5, box_moment, 1e-6);
}

test "setType zeros inverse mass for non-dynamic bodies" {
    var body = cpBody.init(2.0, 4.0);
    body.v = vect.cpv(3.0, 3.0);
    body.w = 2.0;
    body.setType(.static);
    try std.testing.expectEqual(@as(types.cpFloat, 0.0), body.m_inv);
    try std.testing.expectEqual(@as(types.cpFloat, 0.0), body.i_inv);
    try std.testing.expectEqual(vect.cpvzero, body.v);
    try std.testing.expectEqual(@as(types.cpFloat, 0.0), body.w);

    body.setType(.dynamic);
    try std.testing.expectApproxEqAbs(0.5, body.m_inv, 1e-9);
    try std.testing.expectApproxEqAbs(0.25, body.i_inv, 1e-9);
}

test "body tracks attachments for shapes constraints and arbiters" {
    var body_a = cpBody.init(1.0, 1.0);
    var body_b = cpBody.init(1.0, 1.0);

    var shape_a = shape_base.cpShape.init(.circle, &body_a);
    var shape_b = shape_base.cpShape.init(.circle, &body_b);

    body_a.attachShape(&shape_a);
    try std.testing.expectEqual(&shape_a, body_a.shape_list);
    body_a.detachShape(&shape_a);
    try std.testing.expectEqual(@as(?*shape_base.cpShape, null), body_a.shape_list);

    var constraint = constraint_base.cpConstraint.init(&body_a, &body_b);
    body_a.attachConstraint(&constraint);
    body_b.attachConstraint(&constraint);
    try std.testing.expectEqual(&constraint, body_a.constraint_list);
    try std.testing.expectEqual(&constraint, body_b.constraint_list);
    body_a.detachConstraint(&constraint);
    body_b.detachConstraint(&constraint);
    try std.testing.expect(body_a.constraint_list == null and body_b.constraint_list == null);

    var arb = arbiter_mod.cpArbiter.init(&shape_a, &shape_b);
    body_a.attachArbiter(&arb);
    body_b.attachArbiter(&arb);
    try std.testing.expectEqual(&arb, body_a.arbiter_list);
    try std.testing.expectEqual(&arb, body_b.arbiter_list);
    body_a.detachArbiter(&arb);
    body_b.detachArbiter(&arb);
    try std.testing.expect(body_a.arbiter_list == null and body_b.arbiter_list == null);
}
