const std = @import("std");
const types = @import("../core/types.zig");
const body_mod = @import("../space/body.zig");

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

    pub fn preStep(self: *cpConstraint, dt: types.cpFloat) void {
        _ = self;
        _ = dt;
    }

    pub fn applyCachedImpulse(self: *cpConstraint, dt_coef: types.cpFloat) void {
        _ = self;
        _ = dt_coef;
    }

    pub fn applyImpulse(self: *cpConstraint) void {
        _ = self;
    }
};

test "constraint stores references" {
    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(2.0, 2.0);
    const constraint = cpConstraint.init(&body_a, &body_b);
    try std.testing.expect(constraint.a.m == 1.0);
    try std.testing.expect(constraint.b.m == 2.0);
}
