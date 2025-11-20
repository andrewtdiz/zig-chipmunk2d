const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");

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
    user_data: types.cpDataPointer = null,

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

pub fn cpMomentForCircle(mass: types.cpFloat, inner_radius: types.cpFloat, outer_radius: types.cpFloat, offset: vect.cpVect) types.cpFloat {
    const r1_sq = inner_radius * inner_radius;
    const r2_sq = outer_radius * outer_radius;
    return mass * (0.5 * (r1_sq + r2_sq) + vect.cpvlengthsq(offset));
}

pub fn cpMomentForSegment(mass: types.cpFloat, a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) types.cpFloat {
    const length_sq = vect.cpvdistsq(a, b);
    return mass * (length_sq / 12.0 + radius * radius * 0.5);
}

pub fn cpMomentForBox(mass: types.cpFloat, width: types.cpFloat, height: types.cpFloat) types.cpFloat {
    return mass * (width * width + height * height) / 12.0;
}

pub fn cpMomentForBox2(mass: types.cpFloat, bounds: struct { width: types.cpFloat, height: types.cpFloat, offset: vect.cpVect }) types.cpFloat {
    const base = cpMomentForBox(mass, bounds.width, bounds.height);
    return base + mass * vect.cpvlengthsq(bounds.offset);
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

test "moment helpers" {
    const circle_moment = cpMomentForCircle(2.0, 0.0, 3.0, vect.cpvzero);
    try std.testing.expectApproxEqAbs(9.0, circle_moment, 1e-6);

    const segment_moment = cpMomentForSegment(1.0, vect.cpv(-1.0, 0.0), vect.cpv(1.0, 0.0), 0.5);
    try std.testing.expect(segment_moment > 0.0);

    const box_moment = cpMomentForBox(1.5, 2.0, 4.0);
    try std.testing.expectApproxEqAbs(2.5, box_moment, 1e-6);
}
