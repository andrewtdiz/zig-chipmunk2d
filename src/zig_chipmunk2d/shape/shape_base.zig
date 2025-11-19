const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const transform = @import("../core/transform.zig");
const body_mod = @import("../space/body.zig");

pub const cpShapeFilter = struct {
    group: types.cpGroup,
    categories: types.cpBitmask,
    mask: types.cpBitmask,

    pub fn all() cpShapeFilter {
        return .{ .group = types.CP_NO_GROUP, .categories = types.CP_ALL_CATEGORIES, .mask = types.CP_ALL_CATEGORIES };
    }

    pub fn reject(a: cpShapeFilter, b: cpShapeFilter) types.cpBool {
        if (a.group != 0 and a.group == b.group) return true;
        if ((a.categories & b.mask) == 0) return true;
        if ((b.categories & a.mask) == 0) return true;
        return false;
    }
};

pub const ShapeType = enum { circle, segment, poly };

pub const cpShape = struct {
    shape_type: ShapeType,
    body: *body_mod.cpBody,
    bb_value: bb.cpBB = bb.cpBBNew(0.0, 0.0, 0.0, 0.0),
    sensor: types.cpBool = false,
    elasticity: types.cpFloat = 0.0,
    friction: types.cpFloat = 0.7,
    surface_velocity: vect.cpVect = vect.cpvzero,
    filter: cpShapeFilter = cpShapeFilter.all(),
    user_data: types.cpDataPointer = null,

    pub fn init(shape_type: ShapeType, body: *body_mod.cpBody) cpShape {
        return .{ .shape_type = shape_type, .body = body };
    }

    pub fn setBB(self: *cpShape, bounds: bb.cpBB) void {
        self.bb_value = bounds;
    }

    pub fn bbValue(self: cpShape) bb.cpBB {
        return self.bb_value;
    }

    pub fn cacheTransformedBB(self: *cpShape, transform_value: transform.cpTransform) void {
        self.bb_value = transform.cpTransformbBB(transform_value, self.bb_value);
    }
};

pub fn cpAreaForCircle(inner_radius: types.cpFloat, outer_radius: types.cpFloat) types.cpFloat {
    return types.CP_PI * (outer_radius * outer_radius - inner_radius * inner_radius);
}

pub fn cpAreaForSegment(a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) types.cpFloat {
    const length = vect.cpvdist(a, b);
    return types.CP_PI * radius * radius + 2.0 * radius * length;
}

pub fn cpAreaForPoly(vertices: []const vect.cpVect) types.cpFloat {
    var area: types.cpFloat = 0.0;
    if (vertices.len < 3) return 0.0;

    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vertices[i];
        const b_ = vertices[(i + 1) % vertices.len];
        area += vect.cpvcross(a, b_);
    }

    return area * 0.5;
}

pub fn cpCentroidForPoly(vertices: []const vect.cpVect) vect.cpVect {
    var area_acc: types.cpFloat = 0.0;
    var centroid = vect.cpvzero;
    if (vertices.len < 3) return centroid;

    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vertices[i];
        const b_ = vertices[(i + 1) % vertices.len];
        const cross = vect.cpvcross(a, b_);
        area_acc += cross;
        centroid = vect.cpvadd(centroid, vect.cpvmult(vect.cpvadd(a, b_), cross));
    }

    const area = area_acc * 0.5;
    if (area == 0.0) return centroid;

    return vect.cpvmult(centroid, 1.0 / (6.0 * area));
}

pub fn cpMomentForPoly(mass: types.cpFloat, vertices: []const vect.cpVect, offset: vect.cpVect, radius: types.cpFloat) types.cpFloat {
    if (vertices.len < 2) return 0.0;

    var sum1: types.cpFloat = 0.0;
    var sum2: types.cpFloat = 0.0;
    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vect.cpvadd(vertices[i], offset);
        const b_ = vect.cpvadd(vertices[(i + 1) % vertices.len], offset);
        const cross = vect.cpvcross(b_, a);
        const a_sq = vect.cpvdot(a, a);
        const b_sq = vect.cpvdot(b_, b_);

        sum1 += cross * (a_sq + b_sq + vect.cpvdot(a, b_));
        sum2 += cross;
    }

    if (sum2 == 0.0) return 0.0;

    const inertia = (mass / 6.0) * (sum1 / sum2);
    return inertia + mass * radius * radius;
}

test "shape filter rejection" {
    const filterA = cpShapeFilter.all();
    const filterB = cpShapeFilter{ .group = 1, .categories = types.CP_ALL_CATEGORIES, .mask = types.CP_ALL_CATEGORIES };
    try std.testing.expect(cpShapeFilter.reject(filterB, filterB));
    try std.testing.expect(!cpShapeFilter.reject(filterA, filterA));
}

test "polygon helpers" {
    const verts = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(2.0, 0.0), vect.cpv(2.0, 2.0), vect.cpv(0.0, 2.0) };
    const area = cpAreaForPoly(&verts);
    try std.testing.expectApproxEqAbs(4.0, area, 1e-6);
    const centroid = cpCentroidForPoly(&verts);
    try std.testing.expectApproxEqAbs(1.0, centroid.x, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, centroid.y, 1e-6);
}
