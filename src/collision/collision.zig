const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const circle = @import("../shape/circle.zig");
const segment = @import("../shape/segment.zig");
const poly = @import("../shape/poly.zig");
const shape_base = @import("../shape/shape_base.zig");

pub const max_contacts = 2;

pub const Contact = struct {
    point: vect.cpVect,
    normal: vect.cpVect,
    distance: types.cpFloat,
};

pub const CollisionResult = struct {
    contacts: std.BoundedArray(Contact, max_contacts),
    normal: vect.cpVect,

    pub fn empty() CollisionResult {
        return .{ .contacts = std.BoundedArray(Contact, max_contacts).init(0) catch unreachable, .normal = vect.cpvzero };
    }

    pub fn addContact(self: *CollisionResult, contact: Contact) void {
        _ = self.contacts.append(contact) catch {};
        self.normal = contact.normal;
    }

    pub fn contactCount(self: CollisionResult) usize {
        return self.contacts.len;
    }
};

pub fn circleToCircle(a: *const circle.cpCircleShape, b: *const circle.cpCircleShape) CollisionResult {
    var result = CollisionResult.empty();

    const da = vect.cpvadd(a.base.body.p, vect.cpvrotate(a.offset, a.base.body.rotationVector()));
    const db = vect.cpvadd(b.base.body.p, vect.cpvrotate(b.offset, b.base.body.rotationVector()));

    const delta = vect.cpvsub(db, da);
    const distance = vect.cpvlength(delta);
    const combined = a.radius + b.radius;

    if (distance <= combined) {
        const normal = if (distance == 0.0) vect.cpv(1.0, 0.0) else vect.cpvmult(delta, 1.0 / distance);
        const contact_point = vect.cpvadd(da, vect.cpvmult(normal, a.radius));
        result.addContact(.{ .point = contact_point, .normal = normal, .distance = distance - combined });
    }

    return result;
}

fn closestPointOnSegment(p: vect.cpVect, a: vect.cpVect, b_: vect.cpVect) vect.cpVect {
    const ab = vect.cpvsub(b_, a);
    const ab_len_sq = vect.cpvdot(ab, ab);
    if (ab_len_sq == 0) return a;

    const t = types.cpfclamp(vect.cpvdot(vect.cpvsub(p, a), ab) / ab_len_sq, 0.0, 1.0);
    return vect.cpvadd(a, vect.cpvmult(ab, t));
}

pub fn circleToSegment(a: *const circle.cpCircleShape, b: *const segment.cpSegmentShape) CollisionResult {
    var result = CollisionResult.empty();

    const circle_center = vect.cpvadd(a.base.body.p, vect.cpvrotate(a.offset, a.base.body.rotationVector()));
    const rot_b = b.base.body.rotationVector();
    const seg_a = vect.cpvadd(b.base.body.p, vect.cpvrotate(b.a, rot_b));
    const seg_b = vect.cpvadd(b.base.body.p, vect.cpvrotate(b.b, rot_b));

    const closest = closestPointOnSegment(circle_center, seg_a, seg_b);
    const delta = vect.cpvsub(circle_center, closest);
    const distance = vect.cpvlength(delta);
    const combined = a.radius + b.radius;

    if (distance <= combined) {
        const normal = if (distance > 0.0) vect.cpvmult(delta, 1.0 / distance) else vect.cpvrotate(b.normal, rot_b);
        const contact_point = vect.cpvadd(closest, vect.cpvmult(normal, b.radius));
        result.addContact(.{ .point = contact_point, .normal = normal, .distance = distance - combined });
    }

    return result;
}

fn transformPolyVertices(allocator: std.mem.Allocator, shape: *const poly.cpPolyShape) ![]vect.cpVect {
    const rot = shape.base.body.rotationVector();
    const verts = try allocator.alloc(vect.cpVect, shape.vertices.len);
    for (shape.vertices, 0..) |vertex, i| {
        verts[i] = vect.cpvadd(shape.base.body.p, vect.cpvrotate(vertex, rot));
    }
    return verts;
}

fn supportPoint(vertices: []const vect.cpVect, direction: vect.cpVect) vect.cpVect {
    var max_dot: types.cpFloat = -types.CP_INFINITY;
    var support = vect.cpvzero;
    for (vertices) |vertex| {
        const dot = vect.cpvdot(vertex, direction);
        if (dot > max_dot) {
            max_dot = dot;
            support = vertex;
        }
    }
    return support;
}

fn polygonAxisPenetration(center: vect.cpVect, vertices: []const vect.cpVect, radius: types.cpFloat) struct {
    separated: bool,
    distance: types.cpFloat,
    normal: vect.cpVect,
} {
    var best_distance: types.cpFloat = -types.CP_INFINITY;
    var best_normal = vect.cpvzero;

    var i: usize = 0;
    while (i < vertices.len) : (i += 1) {
        const va = vertices[i];
        const vb = vertices[(i + 1) % vertices.len];
        const edge = vect.cpvsub(vb, va);
        const normal = vect.cpvnormalize(vect.cpvperp(edge));

        const dist = vect.cpvdot(normal, center) - vect.cpvdot(normal, va) - radius;
        if (dist > 0.0) {
            return .{ .separated = true, .distance = dist, .normal = normal };
        }

        if (dist > best_distance) {
            best_distance = dist;
            best_normal = normal;
        }
    }

    return .{ .separated = false, .distance = best_distance, .normal = best_normal };
}

pub fn circleToPoly(allocator: std.mem.Allocator, a: *const circle.cpCircleShape, b: *const poly.cpPolyShape) CollisionResult {
    var result = CollisionResult.empty();
    const center = vect.cpvadd(a.base.body.p, vect.cpvrotate(a.offset, a.base.body.rotationVector()));
    const verts = transformPolyVertices(allocator, b) catch return result;

    const projection = polygonAxisPenetration(center, verts, a.radius + b.radius);
    if (projection.separated) return result;

    const contact_point = vect.cpvsub(center, vect.cpvmult(projection.normal, a.radius));
    result.addContact(.{ .point = contact_point, .normal = projection.normal, .distance = projection.distance });
    return result;
}

fn findAxisLeastPenetration(a_verts: []const vect.cpVect, b_verts: []const vect.cpVect) ?struct {
    overlap: types.cpFloat,
    normal: vect.cpVect,
} {
    var best_overlap = types.CP_INFINITY;
    var best_normal = vect.cpvzero;

    var i: usize = 0;
    while (i < a_verts.len) : (i += 1) {
        const va = a_verts[i];
        const vb = a_verts[(i + 1) % a_verts.len];
        const edge = vect.cpvsub(vb, va);
        const axis = vect.cpvnormalize(vect.cpvperp(edge));

        var min_a = types.CP_INFINITY;
        var max_a = -types.CP_INFINITY;
        for (a_verts) |p| {
            const proj = vect.cpvdot(p, axis);
            min_a = types.cpfmin(min_a, proj);
            max_a = types.cpfmax(max_a, proj);
        }

        var min_b = types.CP_INFINITY;
        var max_b = -types.CP_INFINITY;
        for (b_verts) |p| {
            const proj = vect.cpvdot(p, axis);
            min_b = types.cpfmin(min_b, proj);
            max_b = types.cpfmax(max_b, proj);
        }

        const overlap = types.cpfmin(max_a, max_b) - types.cpfmax(min_a, min_b);
        if (overlap < 0.0) return null;

        if (overlap < best_overlap) {
            best_overlap = overlap;
            const direction = vect.cpvsub(polyCentroid(b_verts), polyCentroid(a_verts));
            best_normal = if (vect.cpvdot(direction, axis) < 0.0) vect.cpvneg(axis) else axis;
        }
    }

    return .{ .overlap = best_overlap, .normal = best_normal };
}

fn polyCentroid(vertices: []const vect.cpVect) vect.cpVect {
    var center = vect.cpvzero;
    for (vertices) |v| center = vect.cpvadd(center, v);
    if (vertices.len == 0) return center;
    return vect.cpvmult(center, 1.0 / @as(types.cpFloat, @floatFromInt(vertices.len)));
}

pub fn polyToPoly(allocator: std.mem.Allocator, a: *const poly.cpPolyShape, b: *const poly.cpPolyShape) CollisionResult {
    var result = CollisionResult.empty();

    const verts_a = transformPolyVertices(allocator, a) catch return result;
    const verts_b = transformPolyVertices(allocator, b) catch return result;

    const axis_a = findAxisLeastPenetration(verts_a, verts_b) orelse return result;
    const axis_b = findAxisLeastPenetration(verts_b, verts_a) orelse return result;

    const reference = if (axis_a.overlap < axis_b.overlap) axis_a else .{ .overlap = axis_b.overlap, .normal = vect.cpvneg(axis_b.normal) };

    const point_a = supportPoint(verts_a, vect.cpvneg(reference.normal));
    const point_b = supportPoint(verts_b, reference.normal);
    const contact_point = vect.cpvadd(point_a, vect.cpvmult(vect.cpvsub(point_b, point_a), 0.5));

    result.addContact(.{ .point = contact_point, .normal = reference.normal, .distance = -reference.overlap });
    return result;
}

pub fn bbOverlap(a: *const shape_base.cpShape, b: *const shape_base.cpShape) CollisionResult {
    var result = CollisionResult.empty();
    if (!bb.cpBBIntersects(a.bbValue(), b.bbValue())) return result;

    const delta = vect.cpvsub(b.body.p, a.body.p);
    const normal = if (delta.x == 0.0 and delta.y == 0.0) vect.cpv(1.0, 0.0) else vect.cpvnormalize(delta);
    const contact_point = vect.cpvadd(a.body.p, vect.cpvmult(normal, 0.5 * vect.cpvlength(delta)));
    result.addContact(.{ .point = contact_point, .normal = normal, .distance = 0.0 });
    return result;
}

pub fn collide(allocator: std.mem.Allocator, a: *const shape_base.cpShape, b: *const shape_base.cpShape) CollisionResult {
    return switch (a.shape_type) {
        .circle => switch (b.shape_type) {
            .circle => circleToCircle(@ptrCast(a), @ptrCast(b)),
            .segment => circleToSegment(@ptrCast(a), @ptrCast(b)),
            .poly => circleToPoly(allocator, @ptrCast(a), @ptrCast(b)),
        },
        .segment => switch (b.shape_type) {
            .circle => circleToSegment(@ptrCast(b), @ptrCast(a)),
            .segment => bbOverlap(a, b),
            .poly => circleToPoly(allocator, @ptrCast(b), @ptrCast(a)),
        },
        .poly => switch (b.shape_type) {
            .circle => circleToPoly(allocator, @ptrCast(b), @ptrCast(a)),
            .segment => circleToPoly(allocator, @ptrCast(a), @ptrCast(b)),
            .poly => polyToPoly(allocator, @ptrCast(a), @ptrCast(b)),
        },
    };
}

pub fn testCircleCollisions() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(1.0, 0.0));

    var circle_a = circle.cpCircleShape.init(&body_a, 1.0, vect.cpvzero);
    var circle_b = circle.cpCircleShape.init(&body_b, 1.0, vect.cpvzero);

    const contact = circleToCircle(&circle_a, &circle_b);
    try std.testing.expect(contact.contactCount() == 1);
    const first = contact.contacts.constSlice()[0];
    try std.testing.expect(first.distance < 0.0);
}

pub fn testBBoxFallback() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(0.1, 0.1));

    const verts = [_]vect.cpVect{ vect.cpv(-0.5, -0.5), vect.cpv(0.5, -0.5), vect.cpv(0.5, 0.5) };
    var poly_a = poly.cpPolyShape.init(&body_a, &verts, 0.0);
    var poly_b = poly.cpPolyShape.init(&body_b, &verts, 0.0);

    const result = collide(std.testing.allocator, &poly_a.base, &poly_b.base);
    try std.testing.expect(result.contactCount() == 1);
}

pub fn testCircleSegmentCollision() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(0.5, 0.0));

    var circle_a = circle.cpCircleShape.init(&body_a, 0.5, vect.cpvzero);
    var segment_b = segment.cpSegmentShape.init(&body_b, vect.cpv(-1.0, 0.0), vect.cpv(1.0, 0.0), 0.1);

    const contact = collide(std.testing.allocator, &circle_a.base, &segment_b.base);
    try std.testing.expect(contact.contactCount() == 1);
    try std.testing.expect(contact.contacts.constSlice()[0].distance < 0.0);
}

pub fn testPolyPolyCollision() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(0.5, 0.0));

    const verts = [_]vect.cpVect{ vect.cpv(-0.5, -0.5), vect.cpv(0.5, -0.5), vect.cpv(0.5, 0.5), vect.cpv(-0.5, 0.5) };
    var poly_a = poly.cpPolyShape.init(&body_a, &verts, 0.0);
    var poly_b = poly.cpPolyShape.init(&body_b, &verts, 0.0);

    const result = collide(std.testing.allocator, &poly_a.base, &poly_b.base);
    try std.testing.expect(result.contactCount() == 1);
    try std.testing.expect(result.contacts.constSlice()[0].distance < 0.0);
}

comptime {
    std.testing.refAllDecls(@This());
}
