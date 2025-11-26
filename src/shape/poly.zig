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
        shape.base.setMassInfo(shape_base.cpShapeMassInfoForPoly(0.0, vertices, vect.cpvzero, radius));
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
        return shape_base.cpAreaForPoly(self.vertices, self.radius);
    }
};

pub fn initBox(body: *body_mod.cpBody, width: types.cpFloat, height: types.cpFloat, radius: types.cpFloat) cpPolyShape {
    const hw = width * 0.5;
    const hh = height * 0.5;
    const verts = [_]vect.cpVect{
        vect.cpv(-hw, -hh),
        vect.cpv(-hw, hh),
        vect.cpv(hw, hh),
        vect.cpv(hw, -hh),
    };
    return cpPolyShape.init(body, &verts, radius);
}

pub fn initBoxWithBounds(body: *body_mod.cpBody, bounds: bb.cpBB, radius: types.cpFloat) cpPolyShape {
    const verts = [_]vect.cpVect{
        vect.cpv(bounds.l, bounds.b),
        vect.cpv(bounds.l, bounds.t),
        vect.cpv(bounds.r, bounds.t),
        vect.cpv(bounds.r, bounds.b),
    };
    return cpPolyShape.init(body, &verts, radius);
}

pub fn cpPolyValidate(vertices: []const vect.cpVect) bool {
    if (vertices.len < 3) return false;

    var first_sign: ?types.cpFloat = null;
    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vertices[i];
        const b_ = vertices[(i + 1) % vertices.len];
        const c = vertices[(i + 2) % vertices.len];

        const ab = vect.cpvsub(b_, a);
        const bc = vect.cpvsub(c, b_);
        if (vect.cpvdot(ab, ab) == 0.0 or vect.cpvdot(bc, bc) == 0.0) return false;

        const cross = vect.cpvcross(ab, bc);
        if (cross == 0.0) return false;
        if (first_sign) |sign| {
            if (cross * sign <= 0.0) return false;
        } else {
            first_sign = if (cross > 0.0) 1.0 else -1.0;
        }
    }

    return true;
}

test "poly caches bounding box" {
    var body = body_mod.cpBody.init(1.0, 1.0);
    body.setPosition(vect.cpv(1.0, 1.0));

    const verts = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(1.0, 0.0), vect.cpv(0.0, 1.0) };
    var shape = cpPolyShape.init(&body, &verts, 0.1);
    const bounds = shape.base.bbValue();
    try std.testing.expectApproxEqAbs(0.9, bounds.l, 1e-6);
    try std.testing.expectApproxEqAbs(2.1, bounds.t, 1e-6);
}

test "box helpers build quads" {
    var body = body_mod.cpBody.init(1.0, 1.0);
    var shape = initBox(&body, 4.0, 2.0, 0.0);
    const centroid = shape.centroid();
    try std.testing.expectApproxEqAbs(0.0, centroid.x, 1e-6);
    try std.testing.expectApproxEqAbs(0.0, centroid.y, 1e-6);

    const bounds_shape = initBoxWithBounds(&body, bb.cpBBNew(-1.0, -2.0, 3.0, 2.0), 0.0);
    try std.testing.expect(bounds_shape.vertices.len == 4);
    try std.testing.expect(cpPolyValidate(bounds_shape.vertices));
}

test "cpPolyValidate rejects degenerate and accepts convex loops" {
    const good = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(1.0, 0.0), vect.cpv(1.0, 1.0), vect.cpv(0.0, 1.0) };
    const collinear = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(1.0, 0.0), vect.cpv(2.0, 0.0) };
    const concave = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(1.0, 0.0), vect.cpv(0.5, -1.0), vect.cpv(0.0, 1.0) };

    try std.testing.expect(cpPolyValidate(&good));
    try std.testing.expect(!cpPolyValidate(&collinear));
    try std.testing.expect(!cpPolyValidate(&concave));
}
