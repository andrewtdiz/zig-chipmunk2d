const std = @import("std");
const types = @import("types.zig");

pub const cpVect = struct {
    x: types.cpFloat,
    y: types.cpFloat,
};

pub const cpMat2x2 = struct {
    a: types.cpFloat,
    b: types.cpFloat,
    c: types.cpFloat,
    d: types.cpFloat,
};

pub const cpvzero = cpVect{ .x = 0.0, .y = 0.0 };

pub inline fn cpv(x: types.cpFloat, y: types.cpFloat) cpVect {
    return .{ .x = x, .y = y };
}

pub inline fn cpveql(a: cpVect, b: cpVect) types.cpBool {
    return a.x == b.x and a.y == b.y;
}

pub inline fn cpvadd(a: cpVect, b: cpVect) cpVect {
    return cpv(a.x + b.x, a.y + b.y);
}

pub inline fn cpvsub(a: cpVect, b: cpVect) cpVect {
    return cpv(a.x - b.x, a.y - b.y);
}

pub inline fn cpvneg(v: cpVect) cpVect {
    return cpv(-v.x, -v.y);
}

pub inline fn cpvmult(v: cpVect, scalar: types.cpFloat) cpVect {
    return cpv(v.x * scalar, v.y * scalar);
}

pub inline fn cpvdot(a: cpVect, b: cpVect) types.cpFloat {
    return a.x * b.x + a.y * b.y;
}

pub inline fn cpvcross(a: cpVect, b: cpVect) types.cpFloat {
    return a.x * b.y - a.y * b.x;
}

pub inline fn cpvperp(v: cpVect) cpVect {
    return cpv(-v.y, v.x);
}

pub inline fn cpvrperp(v: cpVect) cpVect {
    return cpv(v.y, -v.x);
}

pub inline fn cpvproject(a: cpVect, b: cpVect) cpVect {
    return cpvmult(b, cpvdot(a, b) / cpvdot(b, b));
}

pub inline fn cpvforangle(radians: types.cpFloat) cpVect {
    return cpv(types.cpfcos(radians), types.cpfsin(radians));
}

pub inline fn cpvtoangle(v: cpVect) types.cpFloat {
    return types.cpfatan2(v.y, v.x);
}

pub inline fn cpvrotate(a: cpVect, b: cpVect) cpVect {
    return cpv(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

pub inline fn cpvunrotate(a: cpVect, b: cpVect) cpVect {
    return cpv(a.x * b.x + a.y * b.y, a.y * b.x - a.x * b.y);
}

pub inline fn cpvlengthsq(v: cpVect) types.cpFloat {
    return cpvdot(v, v);
}

pub inline fn cpvlength(v: cpVect) types.cpFloat {
    return types.cpfsqrt(cpvdot(v, v));
}

pub inline fn cpvlerp(a: cpVect, b: cpVect, t: types.cpFloat) cpVect {
    return cpvadd(cpvmult(a, 1.0 - t), cpvmult(b, t));
}

pub inline fn cpvnormalize(v: cpVect) cpVect {
    return cpvmult(v, 1.0 / (cpvlength(v) + types.CPFLOAT_MIN));
}

pub inline fn cpvslerp(a: cpVect, b: cpVect, t: types.cpFloat) cpVect {
    const dot_value = cpvdot(cpvnormalize(a), cpvnormalize(b));
    const omega = types.cpfacos(types.cpfclamp(dot_value, -1.0, 1.0));

    if (omega < 1e-3) {
        return cpvlerp(a, b, t);
    }

    const denom = 1.0 / types.cpfsin(omega);
    return cpvadd(
        cpvmult(a, types.cpfsin((1.0 - t) * omega) * denom),
        cpvmult(b, types.cpfsin(t * omega) * denom),
    );
}

pub inline fn cpvslerpconst(a: cpVect, b: cpVect, angle: types.cpFloat) cpVect {
    const dot_value = cpvdot(cpvnormalize(a), cpvnormalize(b));
    const omega = types.cpfacos(types.cpfclamp(dot_value, -1.0, 1.0));

    return cpvslerp(a, b, types.cpfmin(angle, omega) / omega);
}

pub inline fn cpvclamp(v: cpVect, max_length: types.cpFloat) cpVect {
    return if (cpvdot(v, v) > max_length * max_length)
        cpvmult(cpvnormalize(v), max_length)
    else
        v;
}

pub inline fn cpvlerpconst(a: cpVect, b: cpVect, distance: types.cpFloat) cpVect {
    return cpvadd(a, cpvclamp(cpvsub(b, a), distance));
}

pub inline fn cpvdist(a: cpVect, b: cpVect) types.cpFloat {
    return cpvlength(cpvsub(a, b));
}

pub inline fn cpvdistsq(a: cpVect, b: cpVect) types.cpFloat {
    return cpvlengthsq(cpvsub(a, b));
}

pub inline fn cpvnear(a: cpVect, b: cpVect, distance: types.cpFloat) types.cpBool {
    return cpvdistsq(a, b) < distance * distance;
}

pub inline fn cpMat2x2New(a: types.cpFloat, b: types.cpFloat, c_: types.cpFloat, d_: types.cpFloat) cpMat2x2 {
    return .{ .a = a, .b = b, .c = c_, .d = d_ };
}

pub inline fn cpMat2x2Transform(m: cpMat2x2, v: cpVect) cpVect {
    return cpv(v.x * m.a + v.y * m.b, v.x * m.c + v.y * m.d);
}

test "cpVect core operations" {
    const a = cpv(1.0, 2.0);
    const b = cpv(4.0, -2.0);

    try std.testing.expect(cpveql(cpvadd(a, b), cpv(5.0, 0.0)));
    try std.testing.expect(cpveql(cpvsub(b, a), cpv(3.0, -4.0)));
    try std.testing.expect(cpveql(cpvperp(a), cpv(-2.0, 1.0)));
    try std.testing.expect(cpvdot(a, b) == 0.0);
    try std.testing.expect(cpvcross(a, b) == -10.0);
    try std.testing.expectApproxEqAbs(5.0, cpvdist(a, b), 1e-9);
    try std.testing.expect(cpvnear(a, cpv(1.5, 2.5), 1.0));
}

test "cpMat2x2 transforms vectors" {
    const m = cpMat2x2New(2.0, 0.0, 0.0, 3.0);
    const v = cpv(1.0, -1.0);
    const transformed = cpMat2x2Transform(m, v);

    try std.testing.expectApproxEqAbs(2.0, transformed.x, 1e-9);
    try std.testing.expectApproxEqAbs(-3.0, transformed.y, 1e-9);
}
