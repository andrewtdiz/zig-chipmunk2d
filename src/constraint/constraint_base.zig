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
    next_a: ?*cpConstraint = null,
    next_b: ?*cpConstraint = null,

    pub fn init(a: *body_mod.cpBody, b: *body_mod.cpBody) cpConstraint {
        return .{ .a = a, .b = b, .error_bias = 0.1 };
    }

    pub fn preStep(self: *cpConstraint, dt: types.cpFloat) void {
        const bias_coef = biasCoefficient(self.error_bias, dt);
        const clamped = types.cpfmin(self.max_bias, bias_coef / dt);
        self.error_bias = clamped;
    }

    pub fn applyCachedImpulse(self: *cpConstraint, dt_coef: types.cpFloat) void {
        // Warm start constraints by scaling any accumulated impulses to the
        // current step. Concrete constraints store the cached impulses in their
        // payload structs and scale them in place.
        _ = dt_coef;
        _ = self;
    }

    pub fn applyImpulse(self: *cpConstraint) void {
        _ = self;
    }

    pub fn nextForBody(self: *cpConstraint, body: *body_mod.cpBody) ?*cpConstraint {
        std.debug.assert(self.a == body or self.b == body);
        return if (self.a == body) self.next_a else self.next_b;
    }

    pub fn nextPtrForBody(self: *cpConstraint, body: *body_mod.cpBody) *?*cpConstraint {
        std.debug.assert(self.a == body or self.b == body);
        return if (self.a == body) &self.next_a else &self.next_b;
    }

    pub fn setNextForBody(self: *cpConstraint, body: *body_mod.cpBody, next: ?*cpConstraint) void {
        std.debug.assert(self.a == body or self.b == body);
        if (self.a == body) {
            self.next_a = next;
        } else {
            self.next_b = next;
        }
    }
};

pub fn biasCoefficient(error_bias: types.cpFloat, dt: types.cpFloat) types.cpFloat {
    return 1.0 - std.math.pow(types.cpFloat, error_bias, dt);
}

test "constraint stores references" {
    var body_a = body_mod.cpBody.init(1.0, 1.0);
    var body_b = body_mod.cpBody.init(2.0, 2.0);
    const constraint = cpConstraint.init(&body_a, &body_b);
    try std.testing.expect(constraint.a.m == 1.0);
    try std.testing.expect(constraint.b.m == 2.0);
}
