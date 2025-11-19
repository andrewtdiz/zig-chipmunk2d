const std = @import("std");
const space_mod = @import("../space/space.zig");
const body_mod = @import("../space/body.zig");
const shape_base = @import("../shape/shape_base.zig");
const constraint_base = @import("../constraint/constraint_base.zig");

pub const cpHastySpace = struct {
    space: space_mod.cpSpace,

    pub fn init(allocator: std.mem.Allocator) !cpHastySpace {
        return .{ .space = try space_mod.cpSpace.init(allocator) };
    }

    pub fn deinit(self: *cpHastySpace) void {
        self.space.deinit();
    }

    pub fn addBody(self: *cpHastySpace, body: *body_mod.cpBody) !void {
        try self.space.addBody(body);
    }

    pub fn addShape(self: *cpHastySpace, shape: *shape_base.cpShape) !void {
        try self.space.addShape(shape);
    }

    pub fn addConstraint(
        self: *cpHastySpace,
        constraint: *constraint_base.cpConstraint,
        ops: space_mod.ConstraintOps,
        payload: ?*anyopaque,
    ) !void {
        try self.space.addConstraint(constraint, ops, payload);
    }

    pub fn step(self: *cpHastySpace, dt: space_mod.types.cpFloat) void {
        // The original cpHastySpace runs portions of the solver in parallel.
        // For now, reuse the serial solver while keeping the API surface identical
        // so callers can swap implementations without changing control flow.
        self.space.step(dt);
    }
};
