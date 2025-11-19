const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const constraint_base = @import("constraint_base.zig");
const body_mod = @import("../space/body.zig");

fn worldAnchor(body: *const body_mod.cpBody, anchor: vect.cpVect) vect.cpVect {
    return vect.cpvadd(body.p, vect.cpvrotate(anchor, body.rotationVector()));
}

fn applyLinearCorrection(a: *body_mod.cpBody, b: *body_mod.cpBody, correction: vect.cpVect) void {
    const total_inv = a.m_inv + b.m_inv;
    if (total_inv == 0.0) return;

    const scale_a = a.m_inv / total_inv;
    const scale_b = b.m_inv / total_inv;

    a.p = vect.cpvsub(a.p, vect.cpvmult(correction, scale_a));
    b.p = vect.cpvadd(b.p, vect.cpvmult(correction, scale_b));
}

fn closestPointOnSegment(p: vect.cpVect, a: vect.cpVect, b: vect.cpVect) vect.cpVect {
    const ab = vect.cpvsub(b, a);
    const ab_len_sq = vect.cpvdot(ab, ab);
    if (ab_len_sq == 0.0) return a;

    const t = types.cpfclamp(vect.cpvdot(vect.cpvsub(p, a), ab) / ab_len_sq, 0.0, 1.0);
    return vect.cpvadd(a, vect.cpvmult(ab, t));
}

fn anchorVelocity(body: *const body_mod.cpBody, offset: vect.cpVect) vect.cpVect {
    return vect.cpvadd(body.v, vect.cpvmult(vect.cpvperp(offset), body.w));
}

pub const PinJoint = struct {
    base: constraint_base.cpConstraint,
    anchor_a: vect.cpVect,
    anchor_b: vect.cpVect,
    dist: types.cpFloat,

    pub fn init(
        a: *body_mod.cpBody,
        b: *body_mod.cpBody,
        anchor_a: vect.cpVect,
        anchor_b: vect.cpVect,
        rest_length: ?types.cpFloat,
    ) PinJoint {
        const world_a = worldAnchor(a, anchor_a);
        const world_b = worldAnchor(b, anchor_b);
        const length = rest_length orelse vect.cpvdist(world_a, world_b);
        return .{ .base = constraint_base.cpConstraint.init(a, b), .anchor_a = anchor_a, .anchor_b = anchor_b, .dist = length };
    }

    pub fn solvePositions(self: *PinJoint) void {
        const world_a = worldAnchor(self.base.a, self.anchor_a);
        const world_b = worldAnchor(self.base.b, self.anchor_b);
        const delta = vect.cpvsub(world_b, world_a);
        const dist = vect.cpvlength(delta);
        if (dist == 0.0) return;

        const n = vect.cpvmult(delta, 1.0 / dist);
        const distance_error = dist - self.dist;
        if (distance_error == 0.0) return;

        const correction = vect.cpvmult(n, distance_error);
        applyLinearCorrection(self.base.a, self.base.b, correction);
    }
};

pub const SlideJoint = struct {
    base: constraint_base.cpConstraint,
    anchor_a: vect.cpVect,
    anchor_b: vect.cpVect,
    min: types.cpFloat,
    max: types.cpFloat,

    pub fn init(
        a: *body_mod.cpBody,
        b: *body_mod.cpBody,
        anchor_a: vect.cpVect,
        anchor_b: vect.cpVect,
        min: types.cpFloat,
        max: types.cpFloat,
    ) SlideJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .anchor_a = anchor_a, .anchor_b = anchor_b, .min = min, .max = max };
    }

    pub fn solvePositions(self: *SlideJoint) void {
        const world_a = worldAnchor(self.base.a, self.anchor_a);
        const world_b = worldAnchor(self.base.b, self.anchor_b);
        const delta = vect.cpvsub(world_b, world_a);
        const dist = vect.cpvlength(delta);
        if (dist == 0.0) return;

        const target = types.cpfclamp(dist, self.min, self.max);
        if (dist == target) return;

        const n = vect.cpvmult(delta, 1.0 / dist);
        const correction = vect.cpvmult(n, dist - target);
        applyLinearCorrection(self.base.a, self.base.b, correction);
    }
};

pub const PivotJoint = struct {
    base: constraint_base.cpConstraint,
    anchor_a: vect.cpVect,
    anchor_b: vect.cpVect,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, anchor_a: vect.cpVect, anchor_b: vect.cpVect) PivotJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .anchor_a = anchor_a, .anchor_b = anchor_b };
    }

    pub fn solvePositions(self: *PivotJoint) void {
        const world_a = worldAnchor(self.base.a, self.anchor_a);
        const world_b = worldAnchor(self.base.b, self.anchor_b);
        const correction = vect.cpvsub(world_b, world_a);
        applyLinearCorrection(self.base.a, self.base.b, correction);
    }
};

pub const GrooveJoint = struct {
    base: constraint_base.cpConstraint,
    groove_a: vect.cpVect,
    groove_b: vect.cpVect,
    anchor_b: vect.cpVect,

    pub fn init(
        a: *body_mod.cpBody,
        b: *body_mod.cpBody,
        groove_a: vect.cpVect,
        groove_b: vect.cpVect,
        anchor_b: vect.cpVect,
    ) GrooveJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .groove_a = groove_a, .groove_b = groove_b, .anchor_b = anchor_b };
    }

    pub fn solvePositions(self: *GrooveJoint) void {
        const world_groove_a = worldAnchor(self.base.a, self.groove_a);
        const world_groove_b = worldAnchor(self.base.a, self.groove_b);
        const target = closestPointOnSegment(worldAnchor(self.base.b, self.anchor_b), world_groove_a, world_groove_b);
        const current = worldAnchor(self.base.b, self.anchor_b);
        const correction = vect.cpvsub(current, target);
        applyLinearCorrection(self.base.a, self.base.b, correction);
    }
};

pub const DampedSpring = struct {
    base: constraint_base.cpConstraint,
    anchor_a: vect.cpVect,
    anchor_b: vect.cpVect,
    rest_length: types.cpFloat,
    stiffness: types.cpFloat,
    damping: types.cpFloat,

    pub fn init(
        a: *body_mod.cpBody,
        b: *body_mod.cpBody,
        anchor_a: vect.cpVect,
        anchor_b: vect.cpVect,
        rest_length: types.cpFloat,
        stiffness: types.cpFloat,
        damping: types.cpFloat,
    ) DampedSpring {
        return .{
            .base = constraint_base.cpConstraint.init(a, b),
            .anchor_a = anchor_a,
            .anchor_b = anchor_b,
            .rest_length = rest_length,
            .stiffness = stiffness,
            .damping = damping,
        };
    }

    pub fn applyForces(self: *DampedSpring) void {
        const ra = vect.cpvrotate(self.anchor_a, self.base.a.rotationVector());
        const rb = vect.cpvrotate(self.anchor_b, self.base.b.rotationVector());
        const world_a = vect.cpvadd(self.base.a.p, ra);
        const world_b = vect.cpvadd(self.base.b.p, rb);

        const delta = vect.cpvsub(world_b, world_a);
        const dist = vect.cpvlength(delta);
        if (dist == 0.0) return;

        const n = vect.cpvmult(delta, 1.0 / dist);
        const rel_vel = vect.cpvdot(vect.cpvsub(anchorVelocity(self.base.b, rb), anchorVelocity(self.base.a, ra)), n);
        const force = (self.rest_length - dist) * self.stiffness - rel_vel * self.damping;

        const total_inv = self.base.a.m_inv + self.base.b.m_inv;
        if (total_inv == 0.0) return;

        const impulse = force / total_inv;
        self.base.a.v = vect.cpvsub(self.base.a.v, vect.cpvmult(n, impulse * self.base.a.m_inv));
        self.base.b.v = vect.cpvadd(self.base.b.v, vect.cpvmult(n, impulse * self.base.b.m_inv));
    }
};

pub const DampedRotarySpring = struct {
    base: constraint_base.cpConstraint,
    rest_angle: types.cpFloat,
    stiffness: types.cpFloat,
    damping: types.cpFloat,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, rest_angle: types.cpFloat, stiffness: types.cpFloat, damping: types.cpFloat) DampedRotarySpring {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .rest_angle = rest_angle, .stiffness = stiffness, .damping = damping };
    }

    pub fn applyTorques(self: *DampedRotarySpring) void {
        const relative_angle = self.base.b.a - self.base.a.a;
        const angular_velocity = self.base.b.w - self.base.a.w;
        const torque = (self.rest_angle - relative_angle) * self.stiffness - angular_velocity * self.damping;

        const denom = self.base.a.i_inv + self.base.b.i_inv;
        if (denom == 0.0) return;

        const impulse = torque / denom;
        self.base.a.w -= impulse * self.base.a.i_inv;
        self.base.b.w += impulse * self.base.b.i_inv;
    }
};

pub const SimpleMotor = struct {
    base: constraint_base.cpConstraint,
    rate: types.cpFloat,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, rate: types.cpFloat) SimpleMotor {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .rate = rate };
    }

    pub fn drive(self: *SimpleMotor) void {
        const diff = self.base.b.w - self.base.a.w;
        const rate_error = self.rate - diff;
        const denom = self.base.a.i_inv + self.base.b.i_inv;
        if (denom == 0.0) return;

        const impulse = rate_error / denom;
        self.base.a.w -= impulse * self.base.a.i_inv;
        self.base.b.w += impulse * self.base.b.i_inv;
    }
};

pub const GearJoint = struct {
    base: constraint_base.cpConstraint,
    phase: types.cpFloat,
    ratio: types.cpFloat,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, phase: types.cpFloat, ratio: types.cpFloat) GearJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .phase = phase, .ratio = ratio };
    }

    pub fn solveAngles(self: *GearJoint) void {
        const relative = self.base.b.a - self.base.a.a * self.ratio;
        const phase_error = relative - self.phase;
        const denom = self.base.b.i_inv + self.base.a.i_inv * self.ratio * self.ratio;
        if (denom == 0.0) return;

        const impulse = phase_error / denom;
        self.base.b.a -= impulse * self.base.b.i_inv;
        self.base.a.a += impulse * self.ratio * self.base.a.i_inv;
    }

    pub fn matchAngularVelocity(self: *GearJoint) void {
        const diff = self.base.b.w - self.base.a.w * self.ratio;
        const denom = self.base.b.i_inv + self.base.a.i_inv * self.ratio * self.ratio;
        if (denom == 0.0) return;

        const impulse = diff / denom;
        self.base.b.w -= impulse * self.base.b.i_inv;
        self.base.a.w += impulse * self.ratio * self.base.a.i_inv;
    }
};

pub const RatchetJoint = struct {
    base: constraint_base.cpConstraint,
    phase: types.cpFloat,
    ratchet: types.cpFloat,
    angle: types.cpFloat,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, phase: types.cpFloat, ratchet: types.cpFloat) RatchetJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .phase = phase, .ratchet = ratchet, .angle = b.a - a.a };
    }

    pub fn applyLimits(self: *RatchetJoint) void {
        if (self.ratchet == 0.0) return;

        const delta = self.base.b.a - self.base.a.a;
        const step = self.phase + types.cpffloor((delta - self.phase) / self.ratchet) * self.ratchet;
        const ratchet_error = delta - step;
        if (ratchet_error == 0.0) return;

        const denom = self.base.a.i_inv + self.base.b.i_inv;
        if (denom == 0.0) return;

        const impulse = ratchet_error / denom;
        self.base.a.a += impulse * self.base.a.i_inv;
        self.base.b.a -= impulse * self.base.b.i_inv;
        self.angle = step;
    }
};

pub const RotaryLimitJoint = struct {
    base: constraint_base.cpConstraint,
    min: types.cpFloat,
    max: types.cpFloat,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody, min: types.cpFloat, max: types.cpFloat) RotaryLimitJoint {
        return .{ .base = constraint_base.cpConstraint.init(a, b), .min = min, .max = max };
    }

    pub fn clampAngles(self: *RotaryLimitJoint) void {
        const delta = self.base.b.a - self.base.a.a;
        if (delta < self.min) {
            const min_error = delta - self.min;
            const denom = self.base.a.i_inv + self.base.b.i_inv;
            if (denom == 0.0) return;
            const impulse = min_error / denom;
            self.base.a.a += impulse * self.base.a.i_inv;
            self.base.b.a -= impulse * self.base.b.i_inv;
        } else if (delta > self.max) {
            const max_error = delta - self.max;
            const denom = self.base.a.i_inv + self.base.b.i_inv;
            if (denom == 0.0) return;
            const impulse = max_error / denom;
            self.base.a.a += impulse * self.base.a.i_inv;
            self.base.b.a -= impulse * self.base.b.i_inv;
        }
    }
};

fn setupBodies() struct { a: body_mod.cpBody, b: body_mod.cpBody } {
    return .{ .a = body_mod.cpBody.init(1.0, 1.0), .b = body_mod.cpBody.init(1.0, 1.0) };
}

test "PinJoint enforces rest length" {
    var bodies = setupBodies();
    bodies.a.setPosition(vect.cpvzero);
    bodies.b.setPosition(vect.cpv(5.0, 0.0));
    var joint = PinJoint.init(&bodies.a, &bodies.b, vect.cpvzero, vect.cpvzero, 2.0);
    joint.solvePositions();

    try std.testing.expectApproxEqAbs(1.5, bodies.a.p.x, 1e-6);
    try std.testing.expectApproxEqAbs(3.5, bodies.b.p.x, 1e-6);
}

test "SlideJoint clamps distance" {
    var bodies = setupBodies();
    bodies.a.setPosition(vect.cpvzero);
    bodies.b.setPosition(vect.cpv(4.0, 0.0));
    var joint = SlideJoint.init(&bodies.a, &bodies.b, vect.cpvzero, vect.cpvzero, 1.0, 2.5);
    joint.solvePositions();

    try std.testing.expectApproxEqAbs(0.75, bodies.a.p.x, 1e-6);
    try std.testing.expectApproxEqAbs(2.75, bodies.b.p.x, 1e-6);
}

test "PivotJoint brings anchors together" {
    var bodies = setupBodies();
    bodies.a.setPosition(vect.cpv(0.0, 0.0));
    bodies.b.setPosition(vect.cpv(2.0, 2.0));
    var joint = PivotJoint.init(&bodies.a, &bodies.b, vect.cpvzero, vect.cpvzero);
    joint.solvePositions();

    try std.testing.expectApproxEqAbs(1.0, bodies.a.p.x, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, bodies.a.p.y, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, bodies.b.p.x, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, bodies.b.p.y, 1e-6);
}

test "GrooveJoint projects anchor onto groove" {
    var bodies = setupBodies();
    bodies.a.setPosition(vect.cpvzero);
    bodies.b.setPosition(vect.cpv(2.0, 0.0));
    var joint = GrooveJoint.init(&bodies.a, &bodies.b, vect.cpv(-1.0, 1.0), vect.cpv(1.0, 1.0), vect.cpvzero);
    joint.solvePositions();

    try std.testing.expectApproxEqAbs(1.0, bodies.a.p.y, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, bodies.b.p.y, 1e-6);
}

test "DampedSpring applies force along axis" {
    var bodies = setupBodies();
    bodies.a.setPosition(vect.cpvzero);
    bodies.b.setPosition(vect.cpv(3.0, 0.0));
    bodies.b.setVelocity(vect.cpv(1.0, 0.0));

    var spring = DampedSpring.init(&bodies.a, &bodies.b, vect.cpvzero, vect.cpvzero, 1.0, 5.0, 0.5);
    spring.applyForces();

    try std.testing.expect(bodies.b.v.x < 1.0);
    try std.testing.expect(bodies.a.v.x > 0.0);
}

test "DampedRotarySpring adjusts angular velocity" {
    var bodies = setupBodies();
    bodies.a.a = 0.0;
    bodies.b.a = types.CP_PI / 2.0;
    bodies.b.w = 1.0;

    var spring = DampedRotarySpring.init(&bodies.a, &bodies.b, 0.0, 10.0, 1.0);
    spring.applyTorques();

    try std.testing.expect(bodies.b.w < 1.0);
    try std.testing.expect(bodies.a.w > 0.0);
}

test "SimpleMotor drives toward rate" {
    var bodies = setupBodies();
    bodies.a.w = 0.0;
    bodies.b.w = 0.0;
    var motor = SimpleMotor.init(&bodies.a, &bodies.b, 2.0);
    motor.drive();

    try std.testing.expectApproxEqAbs(1.0, bodies.b.w - bodies.a.w, 1e-6);
}

test "GearJoint enforces angular ratio" {
    var bodies = setupBodies();
    bodies.a.w = 2.0;
    bodies.b.w = 0.0;
    var gear = GearJoint.init(&bodies.a, &bodies.b, 0.0, 2.0);
    gear.matchAngularVelocity();

    try std.testing.expectApproxEqAbs(bodies.b.w, bodies.a.w * 2.0, 1e-6);
}

test "RatchetJoint snaps to discrete step" {
    var bodies = setupBodies();
    bodies.a.a = 0.0;
    bodies.b.a = 0.75;
    var ratchet = RatchetJoint.init(&bodies.a, &bodies.b, 0.0, 0.5);
    ratchet.applyLimits();

    try std.testing.expectApproxEqAbs(0.5, bodies.b.a - bodies.a.a, 1e-6);
}

test "RotaryLimitJoint clamps angles" {
    var bodies = setupBodies();
    bodies.a.a = 0.0;
    bodies.b.a = 2.0;
    var limit = RotaryLimitJoint.init(&bodies.a, &bodies.b, -0.5, 1.0);
    limit.clampAngles();

    try std.testing.expectApproxEqAbs(1.0, bodies.b.a - bodies.a.a, 1e-6);
}

comptime {
    std.testing.refAllDecls(@This());
}
