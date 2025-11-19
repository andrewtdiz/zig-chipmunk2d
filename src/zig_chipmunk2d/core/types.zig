const std = @import("std");

pub const cpFloat = f64;
pub const cpBool = bool;
pub const cpHashValue = usize;
pub const cpCollisionID = u32;
pub const cpDataPointer = ?*anyopaque;
pub const cpCollisionType = usize;
pub const cpGroup = usize;
pub const cpBitmask = u32;
pub const cpTimestamp = u32;

pub const CP_NO_GROUP: cpGroup = 0;
pub const CP_ALL_CATEGORIES: cpBitmask = ~@as(cpBitmask, 0);
pub const CP_WILDCARD_COLLISION_TYPE: cpCollisionType = ~@as(cpCollisionType, 0);

pub const CP_PI: cpFloat = 3.14159265358979323846264338327950288;
pub const CP_INFINITY: cpFloat = std.math.inf(cpFloat);
pub const CPFLOAT_MIN: cpFloat = std.math.floatMin(cpFloat);

pub inline fn cpfmax(a: cpFloat, b: cpFloat) cpFloat {
    return @max(a, b);
}

pub inline fn cpfmin(a: cpFloat, b: cpFloat) cpFloat {
    return @min(a, b);
}

pub inline fn cpfabs(value: cpFloat) cpFloat {
    return @abs(value);
}

pub inline fn cpfclamp(value: cpFloat, min_value: cpFloat, max_value: cpFloat) cpFloat {
    return cpfmin(cpfmax(value, min_value), max_value);
}

pub inline fn cpfclamp01(value: cpFloat) cpFloat {
    return cpfmax(0.0, cpfmin(value, 1.0));
}

pub inline fn cpflerp(a: cpFloat, b: cpFloat, t: cpFloat) cpFloat {
    return a * (1.0 - t) + b * t;
}

pub inline fn cpflerpconst(a: cpFloat, b: cpFloat, max_delta: cpFloat) cpFloat {
    return a + cpfclamp(b - a, -max_delta, max_delta);
}

pub inline fn cpfsqrt(value: cpFloat) cpFloat {
    return std.math.sqrt(value);
}

pub inline fn cpfsin(value: cpFloat) cpFloat {
    return std.math.sin(value);
}

pub inline fn cpfcos(value: cpFloat) cpFloat {
    return std.math.cos(value);
}

pub inline fn cpfacos(value: cpFloat) cpFloat {
    return std.math.acos(value);
}

pub inline fn cpfatan2(y: cpFloat, x: cpFloat) cpFloat {
    return std.math.atan2(y, x);
}

pub inline fn cpfmod(a: cpFloat, b: cpFloat) cpFloat {
    return @rem(a, b);
}

pub inline fn cpfexp(value: cpFloat) cpFloat {
    return std.math.exp(value);
}

pub inline fn cpfpow(base: cpFloat, exponent: cpFloat) cpFloat {
    return std.math.pow(cpFloat, base, exponent);
}

pub inline fn cpffloor(value: cpFloat) cpFloat {
    return std.math.floor(value);
}

pub inline fn cpfceil(value: cpFloat) cpFloat {
    return std.math.ceil(value);
}

test "cpFloat helpers" {
    try std.testing.expect(cpfmax(1.0, 3.0) == 3.0);
    try std.testing.expect(cpfmin(-2.0, 5.0) == -2.0);
    try std.testing.expect(cpfabs(-4.0) == 4.0);
    try std.testing.expect(cpfclamp(5.0, 0.0, 1.0) == 1.0);
    try std.testing.expect(cpfclamp01(-2.0) == 0.0);
    try std.testing.expect(cpflerp(0.0, 10.0, 0.25) == 2.5);
    try std.testing.expectApproxEqAbs(1.0, cpfsqrt(1.0), 1e-9);
    try std.testing.expectApproxEqAbs(0.0, cpfmod(2.0, 1.0), 1e-9);
}
