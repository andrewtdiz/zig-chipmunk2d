const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const body_mod = @import("../space/body.zig");

pub const ConstraintRow = struct {
    r_a: vect.cpVect = vect.cpvzero,
    r_b: vect.cpVect = vect.cpvzero,
    axis: vect.cpVect = vect.cpv(1.0, 0.0),
    mass: types.cpFloat = 0.0,
    bias: types.cpFloat = 0.0,
    impulse_acc: types.cpFloat = 0.0,
};

fn relativeVelocity(body: *body_mod.cpBody, r: vect.cpVect) vect.cpVect {
    return vect.cpvadd(body.v, vect.cpvmult(vect.cpvperp(r), body.w));
}

fn effectiveMass(a: *body_mod.cpBody, b: *body_mod.cpBody, r_a: vect.cpVect, r_b: vect.cpVect, axis: vect.cpVect) types.cpFloat {
    const cr_a = vect.cpvcross(r_a, axis);
    const cr_b = vect.cpvcross(r_b, axis);
    const denom = a.m_inv + b.m_inv + a.i_inv * cr_a * cr_a + b.i_inv * cr_b * cr_b;
    return if (denom == 0.0) 0.0 else 1.0 / denom;
}

fn biasCoefficient(error_bias: types.cpFloat, dt: types.cpFloat) types.cpFloat {
    return 1.0 - std.math.pow(error_bias, dt);
}

pub const cpConstraint = struct {
    a: *body_mod.cpBody,
    b: *body_mod.cpBody,
    max_force: types.cpFloat = types.CP_INFINITY,
    error_bias: types.cpFloat = 0.0,
    max_bias: types.cpFloat = types.CP_INFINITY,
    user_data: types.cpDataPointer = null,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody) cpConstraint {
        return .{ .a = a, .b = b, .error_bias = 0.1 };
    }

    pub fn configureRow(
        self: *cpConstraint,
        row: *ConstraintRow,
        world_a: vect.cpVect,
        world_b: vect.cpVect,
        axis: vect.cpVect,
        axis_error: types.cpFloat,
        dt: types.cpFloat,
    ) void {
        row.r_a = vect.cpvsub(world_a, self.a.p);
        row.r_b = vect.cpvsub(world_b, self.b.p);
        const axis_len_sq = vect.cpvdot(axis, axis);
        const normalized = if (axis_len_sq > 0.0)
            vect.cpvmult(axis, 1.0 / std.math.sqrt(axis_len_sq))
        else
            vect.cpv(1.0, 0.0);
        row.axis = normalized;
        row.mass = effectiveMass(self.a, self.b, row.r_a, row.r_b, row.axis);
        const bias_coef = biasCoefficient(types.cpfmax(self.error_bias, 0.0), dt);
        const bias = -bias_coef * axis_error / dt;
        row.bias = types.cpfclamp(bias, -self.max_bias, self.max_bias);
    }

    pub fn preStepDistance(
        self: *cpConstraint,
        row: *ConstraintRow,
        world_a: vect.cpVect,
        world_b: vect.cpVect,
        target: types.cpFloat,
        dt: types.cpFloat,
    ) void {
        const delta = vect.cpvsub(world_b, world_a);
        const dist = vect.cpvlength(delta);
        const axis = if (dist > 1e-6) vect.cpvmult(delta, 1.0 / dist) else vect.cpv(1.0, 0.0);
        self.configureRow(row, world_a, world_b, axis, dist - target, dt);
    }

    pub fn applyCachedImpulse(self: *cpConstraint, row: *ConstraintRow, dt_coef: types.cpFloat) void {
        const scaled = row.impulse_acc * dt_coef;
        const delta = scaled - row.impulse_acc;
        row.impulse_acc = scaled;
        if (delta == 0.0) return;
        applyImpulsePair(self.a, self.b, row.r_a, row.r_b, row.axis, delta);
    }

    pub fn applyImpulse(self: *cpConstraint, row: *ConstraintRow, dt: types.cpFloat) void {
        if (row.mass == 0.0) return;
        const va = relativeVelocity(self.a, row.r_a);
        const vb = relativeVelocity(self.b, row.r_b);
        const relative = vect.cpvsub(vb, va);
        const j = -(vect.cpvdot(relative, row.axis) + row.bias) * row.mass;
        const limit = if (self.max_force == types.CP_INFINITY) types.CP_INFINITY else self.max_force * dt;
        const old = row.impulse_acc;
        if (limit == types.CP_INFINITY) {
            row.impulse_acc = old + j;
        } else {
            row.impulse_acc = types.cpfclamp(old + j, -limit, limit);
        }
        const delta = row.impulse_acc - old;
        if (delta == 0.0) return;
        applyImpulsePair(self.a, self.b, row.r_a, row.r_b, row.axis, delta);
    }
};

fn applyImpulsePair(a: *body_mod.cpBody, b: *body_mod.cpBody, r_a: vect.cpVect, r_b: vect.cpVect, axis: vect.cpVect, impulse: types.cpFloat) void {
    const impulse_vec = vect.cpvmult(axis, impulse);
    const point_a = vect.cpvadd(a.p, r_a);
    const point_b = vect.cpvadd(b.p, r_b);
    a.applyImpulseAtWorldPoint(vect.cpvneg(impulse_vec), point_a);
    b.applyImpulseAtWorldPoint(impulse_vec, point_b);
}

test "constraint stores references" {
    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(2.0, 2.0);
    const constraint = cpConstraint.init(&body_a, &body_b);
    try std.testing.expect(constraint.a.m == 1.0);
    try std.testing.expect(constraint.b.m == 2.0);
}
