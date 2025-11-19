const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const body_mod = @import("../space/body.zig");
const shape_base = @import("shape_base.zig");

pub const cpSegmentShape = struct {
    base: shape_base.cpShape,
    a: vect.cpVect,
    b: vect.cpVect,
    normal: vect.cpVect,
    radius: types.cpFloat,

    pub fn init(body: *body_mod.cpBody, a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) cpSegmentShape {
        const delta = vect.cpvsub(b, a);
        const normal = vect.cpvnormalize(vect.cpvperp(delta));
        var shape = cpSegmentShape{ .base = shape_base.cpShape.init(.segment, body), .a = a, .b = b, .normal = normal, .radius = radius };
        shape.cacheBB();
        return shape;
    }

    pub fn cacheBB(self: *cpSegmentShape) void {
        const rot = self.base.body.rotationVector();
        const ta = vect.cpvadd(self.base.body.p, vect.cpvrotate(self.a, rot));
        const tb = vect.cpvadd(self.base.body.p, vect.cpvrotate(self.b, rot));
        const bounds = bb.cpBBNew(
            types.cpfmin(ta.x, tb.x) - self.radius,
            types.cpfmin(ta.y, tb.y) - self.radius,
            types.cpfmax(ta.x, tb.x) + self.radius,
            types.cpfmax(ta.y, tb.y) + self.radius,
        );
        self.base.setBB(bounds);
    }
};

test "segment caches bounding box" {
    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(0.0, 0.0));
    body.a = types.CP_PI / 2.0;

    var shape = cpSegmentShape.init(&body, vect.cpv(-1.0, 0.0), vect.cpv(1.0, 0.0), 0.25);
    const bounds = shape.base.bbValue();
    try std.testing.expectApproxEqAbs(-0.25, bounds.l, 1e-6);
    try std.testing.expectApproxEqAbs(0.25, bounds.r, 1e-6);
    try std.testing.expectApproxEqAbs(-1.25, bounds.b, 1e-6);
}
