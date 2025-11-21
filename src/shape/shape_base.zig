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

pub const cpShapeMassInfo = struct {
    m: types.cpFloat,
    i: types.cpFloat,
    cog: vect.cpVect,
    area: types.cpFloat,

    pub fn init(mass: types.cpFloat, moment: types.cpFloat, cog: vect.cpVect, area: types.cpFloat) cpShapeMassInfo {
        return .{ .m = mass, .i = moment, .cog = cog, .area = area };
    }
};

pub fn cpShapeMassInfoForCircle(mass: types.cpFloat, inner_radius: types.cpFloat, outer_radius: types.cpFloat, offset: vect.cpVect) cpShapeMassInfo {
    return cpShapeMassInfo.init(mass, body_mod.cpMomentForCircle(mass, inner_radius, outer_radius, offset), offset, cpAreaForCircle(inner_radius, outer_radius));
}

pub fn cpShapeMassInfoForSegment(mass: types.cpFloat, a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) cpShapeMassInfo {
    const cog = vect.cpvlerp(a, b, 0.5);
    return cpShapeMassInfo.init(mass, body_mod.cpMomentForSegment(mass, a, b, radius), cog, cpAreaForSegment(a, b, radius));
}

pub fn cpShapeMassInfoForPoly(mass: types.cpFloat, vertices: []const vect.cpVect, offset: vect.cpVect, radius: types.cpFloat) cpShapeMassInfo {
    // Compute centroid with offset applied without extra allocations.
    var area_acc: types.cpFloat = 0.0;
    var centroid = vect.cpvzero;
    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vect.cpvadd(vertices[i], offset);
        const b_ = vect.cpvadd(vertices[(i + 1) % vertices.len], offset);
        const cross = vect.cpvcross(a, b_);
        area_acc += cross;
        centroid = vect.cpvadd(centroid, vect.cpvmult(vect.cpvadd(a, b_), cross));
    }
    const area = area_acc * 0.5;
    const cog = if (area == 0.0) vect.cpvzero else vect.cpvmult(centroid, 1.0 / (6.0 * area));
    return cpShapeMassInfo.init(mass, cpMomentForPoly(mass, vertices, offset, radius), cog, cpAreaForPoly(vertices, radius));
}

pub const cpShape = struct {
    shape_type: ShapeType,
    body: *body_mod.cpBody,
    bb_value: bb.cpBB = bb.cpBBNew(0.0, 0.0, 0.0, 0.0),
    sensor: types.cpBool = false,
    elasticity: types.cpFloat = 0.0,
    friction: types.cpFloat = 0.7,
    surface_velocity: vect.cpVect = vect.cpvzero,
    collision_type: types.cpCollisionType = 0,
    filter: cpShapeFilter = cpShapeFilter.all(),
    user_data: types.cpDataPointer = null,
    mass_info: cpShapeMassInfo = cpShapeMassInfo.init(0.0, 0.0, vect.cpvzero, 0.0),

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

    pub fn setMassInfo(self: *cpShape, info: cpShapeMassInfo) void {
        self.mass_info = info;
    }

    pub fn setMass(self: *cpShape, mass: types.cpFloat) void {
        self.mass_info.m = mass;
    }

    pub fn setDensity(self: *cpShape, density: types.cpFloat) void {
        self.mass_info.m = density * self.mass_info.area;
    }
};

pub fn cpAreaForCircle(inner_radius: types.cpFloat, outer_radius: types.cpFloat) types.cpFloat {
    return types.CP_PI * types.cpfabs(outer_radius * outer_radius - inner_radius * inner_radius);
}

pub fn cpAreaForSegment(a: vect.cpVect, b: vect.cpVect, radius: types.cpFloat) types.cpFloat {
    const length = vect.cpvdist(a, b);
    return radius * (types.CP_PI * radius + 2.0 * length);
}

pub fn cpAreaForPoly(vertices: []const vect.cpVect, radius: types.cpFloat) types.cpFloat {
    var area: types.cpFloat = 0.0;
    var perimeter: types.cpFloat = 0.0;
    if (vertices.len < 3) return 0.0;

    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const a = vertices[i];
        const b_ = vertices[(i + 1) % vertices.len];
        area += vect.cpvcross(a, b_);
        perimeter += vect.cpvdist(a, b_);
    }

    const poly_area = area * 0.5;
    return radius * (types.CP_PI * types.cpfabs(radius) + perimeter) + poly_area;
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

pub fn cpMomentForPoly(mass: types.cpFloat, vertices: []const vect.cpVect, offset: vect.cpVect, _: types.cpFloat) types.cpFloat {
    if (vertices.len == 2) return body_mod.cpMomentForSegment(mass, vertices[0], vertices[1], 0.0);
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

    return (mass * sum1) / (6.0 * sum2);
}

pub fn cpLoopIndexes(vertices: []const vect.cpVect) struct { start: usize, end: usize } {
    var start: usize = 0;
    var end: usize = 0;
    var min = vertices[0];
    var max = vertices[0];

    var i: usize = 1;
    while (i < vertices.len) : (i += 1) {
        const v = vertices[i];
        if (v.x < min.x or (v.x == min.x and v.y < min.y)) {
            min = v;
            start = i;
        } else if (v.x > max.x or (v.x == max.x and v.y > max.y)) {
            max = v;
            end = i;
        }
    }

    return .{ .start = start, .end = end };
}

fn qHullPartition(verts: []vect.cpVect, a: vect.cpVect, b: vect.cpVect, tol: types.cpFloat) usize {
    if (verts.len == 0) return 0;

    var max_val: types.cpFloat = 0.0;
    var pivot: usize = 0;

    const delta = vect.cpvsub(b, a);
    const value_tol = tol * vect.cpvlength(delta);

    var head: usize = 0;
    var tail: usize = verts.len - 1;
    while (head <= tail) {
        const value = vect.cpvcross(vect.cpvsub(verts[head], a), delta);
        if (value > value_tol) {
            if (value > max_val) {
                max_val = value;
                pivot = head;
            }
            head += 1;
        } else {
            std.mem.swap(vect.cpVect, &verts[head], &verts[tail]);
            if (tail == 0) break;
            tail -= 1;
        }
    }

    if (pivot != 0) std.mem.swap(vect.cpVect, &verts[0], &verts[pivot]);
    return head;
}

fn qHullReduce(tol: types.cpFloat, verts: []vect.cpVect, a: vect.cpVect, pivot: vect.cpVect, b: vect.cpVect, result: []vect.cpVect) usize {
    if (verts.len == 0) {
        result[0] = pivot;
        return 1;
    }

    const left_count = qHullPartition(verts, a, pivot, tol);
    var index: usize = 0;
    if (left_count > 1 and verts.len >= 1) {
        index = qHullReduce(tol, verts[1..left_count], a, verts[0], pivot, result);
    }

    result[index] = pivot;
    index += 1;

    const right_slice = verts[left_count..];
    if (right_slice.len > 0) {
        const right_count = qHullPartition(right_slice, pivot, b, tol);
        if (right_count > 1 and right_slice.len >= 1) {
            index += qHullReduce(tol, right_slice[1..right_count], pivot, right_slice[0], b, result[index..]);
        }
    }

    return index;
}

pub fn cpConvexHull(vertices: []const vect.cpVect, result: []vect.cpVect, first_out: ?*usize, tol: types.cpFloat) usize {
    if (vertices.len == 0 or result.len == 0) return 0;
    const count = @min(vertices.len, result.len);
    std.mem.copy(vect.cpVect, result[0..count], vertices[0..count]);

    const indices = cpLoopIndexes(vertices[0..count]);
    if (indices.start == indices.end) {
        if (first_out) |ptr| ptr.* = 0;
        result[0] = vertices[indices.start];
        return 1;
    }

    std.mem.swap(vect.cpVect, &result[0], &result[indices.start]);
    const swap_index = if (indices.end == 0) indices.start else indices.end;
    std.mem.swap(vect.cpVect, &result[1], &result[swap_index]);

    const a = result[0];
    const b = result[1];

    if (first_out) |ptr| ptr.* = indices.start;
    return qHullReduce(tol, result[2..count], a, b, a, result[1..count]) + 1;
}

test "shape filter rejection" {
    const filterA = cpShapeFilter.all();
    const filterB = cpShapeFilter{ .group = 1, .categories = types.CP_ALL_CATEGORIES, .mask = types.CP_ALL_CATEGORIES };
    try std.testing.expect(cpShapeFilter.reject(filterB, filterB));
    try std.testing.expect(!cpShapeFilter.reject(filterA, filterA));
}

test "polygon helpers" {
    const verts = [_]vect.cpVect{ vect.cpv(0.0, 0.0), vect.cpv(2.0, 0.0), vect.cpv(2.0, 2.0), vect.cpv(0.0, 2.0) };
    const area = cpAreaForPoly(&verts, 0.0);
    try std.testing.expectApproxEqAbs(4.0, area, 1e-6);
    const centroid = cpCentroidForPoly(&verts);
    try std.testing.expectApproxEqAbs(1.0, centroid.x, 1e-6);
    try std.testing.expectApproxEqAbs(1.0, centroid.y, 1e-6);
}
