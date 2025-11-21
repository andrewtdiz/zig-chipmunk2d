const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const body_mod = @import("../space/body.zig");
const shape_base = @import("shape_base.zig");

pub const cpCircleShape = struct {
    base: shape_base.cpShape,
    radius: types.cpFloat,
    offset: vect.cpVect,

    pub fn init(body: *body_mod.cpBody, radius: types.cpFloat, offset: vect.cpVect) cpCircleShape {
        var shape = cpCircleShape{ .base = shape_base.cpShape.init(.circle, body), .radius = radius, .offset = offset };
        shape.base.setMassInfo(shape_base.cpShapeMassInfoForCircle(0.0, 0.0, radius, offset));
        shape.cacheBB();
        return shape;
    }

    pub fn cacheBB(self: *cpCircleShape) void {
        const rot = self.base.body.rotationVector();
        const center = vect.cpvadd(self.base.body.p, vect.cpvrotate(self.offset, rot));
        self.base.setBB(bb.cpBBNewForCircle(center, self.radius));
    }
};

test "circle caches bounding box" {
    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(2.0, 3.0));

    var shape = cpCircleShape.init(&body, 1.5, vect.cpv(1.0, 0.0));
    const bounds = shape.base.bbValue();
    try std.testing.expectApproxEqAbs(2.5, bounds.l, 1e-6);
    try std.testing.expectApproxEqAbs(4.5, bounds.r, 1e-6);
}
