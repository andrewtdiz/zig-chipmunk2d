const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const body_mod = @import("../space/body.zig");
const shape_base = @import("shape_base.zig");

pub const cpPolyShape = struct {
    base: shape_base.cpShape,
    vertices: []const vect.cpVect,
    radius: types.cpFloat,

    pub fn init(body: *body_mod.cpBody, vertices: []const vect.cpVect, radius: types.cpFloat) cpPolyShape {
        var shape = cpPolyShape{ .base = shape_base.cpShape.init(.poly, body), .vertices = vertices, .radius = radius };
        shape.cacheBB();
        return shape;
    }

    pub fn cacheBB(self: *cpPolyShape) void {
        const rot = self.base.body.rotationVector();
        var min_x = types.CP_INFINITY;
        var max_x = -types.CP_INFINITY;
        var min_y = types.CP_INFINITY;
        var max_y = -types.CP_INFINITY;

        for (self.vertices) |vertex| {
            const world = vect.cpvadd(self.base.body.p, vect.cpvrotate(vertex, rot));
            min_x = types.cpfmin(min_x, world.x);
            max_x = types.cpfmax(max_x, world.x);
            min_y = types.cpfmin(min_y, world.y);
            max_y = types.cpfmax(max_y, world.y);
        }

        self.base.setBB(bb.cpBBNew(min_x - self.radius, min_y - self.radius, max_x + self.radius, max_y + self.radius));
    }

    pub fn centroid(self: cpPolyShape) vect.cpVect {
        return shape_base.cpCentroidForPoly(self.vertices);
    }

    pub fn area(self: cpPolyShape) types.cpFloat {
        return shape_base.cpAreaForPoly(self.vertices);
    }
};

test "poly caches bounding box" {
    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(1.0, 1.0));

    const verts = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(1.0, 0.0), vect.cpv(0.0, 1.0) };
    var shape = cpPolyShape.init(&body, &verts, 0.1);
    const bounds = shape.base.bbValue();
    try std.testing.expectApproxEqAbs(0.9, bounds.l, 1e-6);
    try std.testing.expectApproxEqAbs(2.1, bounds.t, 1e-6);
}
