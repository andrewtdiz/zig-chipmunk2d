const std = @import("std");
const vect = @import("../core/vect.zig");
const types = @import("../core/types.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("collision.zig");

pub const cpArbiter = struct {
    shape_a: *shape_base.cpShape,
    shape_b: *shape_base.cpShape,
    collision_id: types.cpCollisionID = 0,
    contacts: std.BoundedArray(collision.Contact, 4),

    pub fn init(shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) cpArbiter {
        return .{
            .shape_a = shape_a,
            .shape_b = shape_b,
            .contacts = std.BoundedArray(collision.Contact, 4).init(0) catch unreachable,
        };
    }

    pub fn clear(self: *cpArbiter) void {
        self.contacts.len = 0;
    }

    pub fn addContact(self: *cpArbiter, contact: collision.Contact) void {
        _ = self.contacts.append(contact) catch {};
    }

    pub fn contactCount(self: cpArbiter) usize {
        return self.contacts.len;
    }

    pub fn penetrationDepth(self: cpArbiter) types.cpFloat {
        var depth: types.cpFloat = 0.0;
        for (self.contacts.constSlice()) |contact| {
            if (contact.distance < 0.0) {
                depth += -contact.distance;
            }
        }
        return depth;
    }
};

pub fn testArbiterAccumulation() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    var shape_a = shape_base.cpShape.init(.circle, &body_a);
    var shape_b = shape_base.cpShape.init(.circle, &body_b);

    var arbiter = cpArbiter.init(&shape_a, &shape_b);
    try std.testing.expectEqual(@as(usize, 0), arbiter.contactCount());

    arbiter.addContact(.{ .point = vect.cpvzero, .normal = vect.cpv(1.0, 0.0), .distance = -0.25, .hash = 0 });
    arbiter.addContact(.{ .point = vect.cpv(0.0, 1.0), .normal = vect.cpv(0.0, 1.0), .distance = 0.1, .hash = 0 });

    try std.testing.expectEqual(@as(usize, 2), arbiter.contactCount());
    try std.testing.expectApproxEqAbs(0.25, arbiter.penetrationDepth(), 1e-6);

    arbiter.clear();
    try std.testing.expectEqual(@as(usize, 0), arbiter.contactCount());
}

comptime {
    std.testing.refAllDecls(@This());
}
