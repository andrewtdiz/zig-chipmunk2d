const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const circle = @import("../shape/circle.zig");
const segment = @import("../shape/segment.zig");
const poly = @import("../shape/poly.zig");
const shape_base = @import("../shape/shape_base.zig");

pub const max_contacts = 4;

const MAX_GJK_ITERATIONS = 30;
const MAX_EPA_ITERATIONS = 30;
const WARN_GJK_ITERATIONS = 20;
const WARN_EPA_ITERATIONS = 20;
const MAX_EPA_HULL = 64;

const PairKey = u128;
var collision_id_cache = std.AutoHashMapUnmanaged(PairKey, types.cpCollisionID){};

fn pairKey(a: *const shape_base.cpShape, b: *const shape_base.cpShape) PairKey {
    const addr_a: u64 = @intFromPtr(a);
    const addr_b: u64 = @intFromPtr(b);
    const lo = if (addr_a < addr_b) addr_a else addr_b;
    const hi = if (addr_a < addr_b) addr_b else addr_a;
    return (@as(PairKey, hi) << 64) | lo;
}

fn cachedCollisionId(key: PairKey) types.cpCollisionID {
    return collision_id_cache.get(key) orelse 0;
}

fn storeCollisionId(key: PairKey, id: types.cpCollisionID) void {
    if (collision_id_cache.getPtr(key)) |existing| {
        existing.* = id;
        return;
    }
    collision_id_cache.put(std.heap.page_allocator, key, id) catch {};
}

pub const Contact = struct {
    point: vect.cpVect,
    normal: vect.cpVect,
    distance: types.cpFloat,
    hash: types.cpHashValue = 0,
};

pub const CollisionResult = struct {
    contacts: std.BoundedArray(Contact, 4),
    normal: vect.cpVect,
    id: types.cpCollisionID = 0,

    pub fn empty() CollisionResult {
        return .{ .contacts = std.BoundedArray(Contact, 4).init(0) catch unreachable, .normal = vect.cpvzero, .id = 0 };
    }

    pub fn addContact(self: *CollisionResult, contact: Contact) void {
        _ = self.contacts.append(contact) catch {};
        self.normal = contact.normal;
    }

    pub fn contactCount(self: CollisionResult) usize {
        return self.contacts.len;
    }
};

const SupportPoint = struct {
    p: vect.cpVect,
    index: types.cpCollisionID,
};

fn supportPointNew(p: vect.cpVect, index: types.cpCollisionID) SupportPoint {
    return .{ .p = p, .index = index };
}

const MinkowskiPoint = struct {
    a: vect.cpVect,
    b: vect.cpVect,
    ab: vect.cpVect,
    id: types.cpCollisionID,
};

fn minkowskiPointNew(a: SupportPoint, b: SupportPoint) MinkowskiPoint {
    const id = ((a.index & 0xFF) << 8) | (b.index & 0xFF);
    return .{ .a = a.p, .b = b.p, .ab = vect.cpvsub(b.p, a.p), .id = id };
}

const ClosestPoints = struct {
    a: vect.cpVect,
    b: vect.cpVect,
    n: vect.cpVect,
    d: types.cpFloat,
    id: types.cpCollisionID,
};

fn closestT(a: vect.cpVect, b: vect.cpVect) types.cpFloat {
    const delta = vect.cpvsub(b, a);
    const denom = vect.cpvlengthsq(delta) + types.CPFLOAT_MIN;
    return -types.cpfclamp(vect.cpvdot(delta, vect.cpvadd(a, b)) / denom, -1.0, 1.0);
}

fn lerpT(a: vect.cpVect, b: vect.cpVect, t: types.cpFloat) vect.cpVect {
    const ht = 0.5 * t;
    return vect.cpvadd(vect.cpvmult(a, 0.5 - ht), vect.cpvmult(b, 0.5 + ht));
}

fn closestPointsNew(v0: MinkowskiPoint, v1: MinkowskiPoint) ClosestPoints {
    const t = closestT(v0.ab, v1.ab);
    const p = lerpT(v0.ab, v1.ab, t);

    const pa = lerpT(v0.a, v1.a, t);
    const pb = lerpT(v0.b, v1.b, t);
    const id: types.cpCollisionID = ((v0.id & 0xFFFF) << 16) | (v1.id & 0xFFFF);

    const delta = vect.cpvsub(v1.ab, v0.ab);
    var n = vect.cpvnormalize(vect.cpvrperp(delta));
    const d = vect.cpvdot(n, p);

    if (d <= 0.0 or (-1.0 < t and t < 1.0)) {
        return .{ .a = pa, .b = pb, .n = n, .d = d, .id = id };
    }

    const dist = vect.cpvlength(p);
    n = if (dist > 0.0) vect.cpvmult(p, 1.0 / dist) else vect.cpv(1.0, 0.0);
    return .{ .a = pa, .b = pb, .n = n, .d = dist, .id = id };
}

fn closestDist(v0: vect.cpVect, v1: vect.cpVect) types.cpFloat {
    return vect.cpvlengthsq(lerpT(v0, v1, closestT(v0, v1)));
}

fn cpCheckPointGreater(a: vect.cpVect, b: vect.cpVect, c: vect.cpVect) bool {
    return (b.y - a.y) * (a.x + b.x - 2.0 * c.x) > (b.x - a.x) * (a.y + b.y - 2.0 * c.y);
}

fn cpCheckAxis(v0: vect.cpVect, v1: vect.cpVect, p: vect.cpVect, n: vect.cpVect) bool {
    return vect.cpvdot(p, n) <= types.cpfmax(vect.cpvdot(v0, n), vect.cpvdot(v1, n));
}

const CircleProxy = struct {
    shape: *const circle.cpCircleShape,
    center: vect.cpVect,
    radius: types.cpFloat,
};

fn buildCircleProxy(shape: *const circle.cpCircleShape) CircleProxy {
    const rot = shape.base.body.rotationVector();
    const center = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.offset, rot));
    return .{ .shape = shape, .center = center, .radius = shape.radius };
}

const SegmentProxy = struct {
    shape: *const segment.cpSegmentShape,
    a: vect.cpVect,
    b: vect.cpVect,
    normal: vect.cpVect,
    radius: types.cpFloat,
};

fn buildSegmentProxy(shape: *const segment.cpSegmentShape) SegmentProxy {
    const rot = shape.base.body.rotationVector();
    const a_world = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.a, rot));
    const b_world = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.b, rot));
    return .{
        .shape = shape,
        .a = a_world,
        .b = b_world,
        .normal = vect.cpvnormalize(vect.cpvrotate(shape.normal, rot)),
        .radius = shape.radius,
    };
}

const PolyProxy = struct {
    shape: *const poly.cpPolyShape,
    position: vect.cpVect,
    rotation: vect.cpVect,
    vertices: []const vect.cpVect,
    radius: types.cpFloat,
};

fn buildPolyProxy(shape: *const poly.cpPolyShape) PolyProxy {
    return .{
        .shape = shape,
        .position = shape.base.body.p,
        .rotation = shape.base.body.rotationVector(),
        .vertices = shape.vertices,
        .radius = shape.radius,
    };
}

fn polyVertexWorld(proxy: PolyProxy, index: usize) vect.cpVect {
    if (proxy.vertices.len == 0) return proxy.position;
    const idx = index % proxy.vertices.len;
    return vect.cpvadd(proxy.position, vect.cpvrotate(proxy.vertices[idx], proxy.rotation));
}

const SupportShape = union(enum) {
    circle: CircleProxy,
    segment: SegmentProxy,
    poly: PolyProxy,
};

fn shapeBase(shape: SupportShape) *const shape_base.cpShape {
    return switch (shape) {
        .circle => |proxy| &proxy.shape.base,
        .segment => |proxy| &proxy.shape.base,
        .poly => |proxy| &proxy.shape.base,
    };
}

fn shapeRadius(shape: SupportShape) types.cpFloat {
    return switch (shape) {
        .circle => |proxy| proxy.radius,
        .segment => |proxy| proxy.radius,
        .poly => |proxy| proxy.radius,
    };
}

fn support(shape: SupportShape, direction: vect.cpVect) SupportPoint {
    return switch (shape) {
        .circle => |proxy| supportPointNew(proxy.center, 0),
        .segment => |proxy| blk: {
            const da = vect.cpvdot(proxy.a, direction);
            const db = vect.cpvdot(proxy.b, direction);
            break :blk if (da > db) supportPointNew(proxy.a, 0) else supportPointNew(proxy.b, 1);
        },
        .poly => |proxy| blk: {
            var max_dot = -types.CP_INFINITY;
            var index: usize = 0;
            for (proxy.vertices, 0..) |_, i| {
                const world = polyVertexWorld(proxy, i);
                const dot = vect.cpvdot(world, direction);
                if (dot > max_dot) {
                    max_dot = dot;
                    index = i;
                }
            }
            break :blk supportPointNew(polyVertexWorld(proxy, index), @intCast(index & 0xFF));
        },
    };
}

fn shapePoint(shape: SupportShape, index: usize) SupportPoint {
    return switch (shape) {
        .circle => |proxy| supportPointNew(proxy.center, 0),
        .segment => |proxy| if (index & 1 == 0)
            supportPointNew(proxy.a, 0)
        else
            supportPointNew(proxy.b, 1),
        .poly => |proxy| supportPointNew(polyVertexWorld(proxy, if (proxy.vertices.len == 0) 0 else index % proxy.vertices.len), @intCast(index & 0xFF)),
    };
}

const SupportContext = struct {
    shape1: SupportShape,
    shape2: SupportShape,
};

fn Support(ctx: *const SupportContext, direction: vect.cpVect) MinkowskiPoint {
    const a = support(ctx.shape1, vect.cpvneg(direction));
    const b = support(ctx.shape2, direction);
    return minkowskiPointNew(a, b);
}
const MinkowskiBuffer = struct {
    buf: [MAX_EPA_HULL]MinkowskiPoint = undefined,
    len: usize = 0,

    fn append(self: *MinkowskiBuffer, value: MinkowskiPoint) !void {
        if (self.len >= MAX_EPA_HULL) return error.OutOfSpace;
        self.buf[self.len] = value;
        self.len += 1;
    }

    fn items(self: *const MinkowskiBuffer) []const MinkowskiPoint {
        return self.buf[0..self.len];
    }

    fn last(self: *const MinkowskiBuffer) MinkowskiPoint {
        return self.buf[self.len - 1];
    }
};

fn EPA(ctx: *const SupportContext, v0: MinkowskiPoint, v1: MinkowskiPoint, v2: MinkowskiPoint) ClosestPoints {
    var hull = MinkowskiBuffer{};
    hull.append(v0) catch return closestPointsNew(v0, v1);
    hull.append(v1) catch return closestPointsNew(v0, v1);
    hull.append(v2) catch return closestPointsNew(v0, v1);

    var iteration: usize = 1;
    while (true) : (iteration += 1) {
        var mini: usize = 0;
        var min_dist = types.CP_INFINITY;
        const count = hull.len;

        var j: usize = 0;
        var prev: usize = if (count == 0) 0 else count - 1;
        while (j < count) : (j += 1) {
            const dist = closestDist(hull.items()[prev].ab, hull.items()[j].ab);
            if (dist < min_dist) {
                min_dist = dist;
                mini = prev;
            }
            prev = j;
        }

        const a = hull.items()[mini];
        const b = hull.items()[(mini + 1) % count];
        const n = vect.cpvperp(vect.cpvsub(b.ab, a.ab));
        const p = Support(ctx, n);

        const duplicate = p.id == a.id or p.id == b.id;
        if (!duplicate and cpCheckPointGreater(a.ab, b.ab, p.ab) and iteration < MAX_EPA_ITERATIONS and count < MAX_EPA_HULL - 1) {
            var rebuilt = MinkowskiBuffer{};
            rebuilt.append(p) catch break;

            var i: usize = 0;
            while (i < count) : (i += 1) {
                const idx = (mini + 1 + i) % count;
                const last = rebuilt.last();
                const current = hull.items()[idx];
                const next = if (i + 1 < count) hull.items()[(idx + 1) % count] else p;
                if (cpCheckPointGreater(last.ab, next.ab, current.ab)) {
                    rebuilt.append(current) catch break;
                }
            }

            hull = rebuilt;
            continue;
        }

        if (iteration >= WARN_EPA_ITERATIONS) {
            std.log.warn("High EPA iterations: {}", .{iteration});
        }
        return closestPointsNew(a, b);
    }
}

fn GJKRecurse(ctx: *const SupportContext, v0: MinkowskiPoint, v1: MinkowskiPoint, iteration: usize) ClosestPoints {
    if (iteration > MAX_GJK_ITERATIONS) {
        if (iteration >= WARN_GJK_ITERATIONS) {
            std.log.warn("High GJK iterations: {}", .{iteration});
        }
        return closestPointsNew(v0, v1);
    }

    if (cpCheckPointGreater(v1.ab, v0.ab, vect.cpvzero)) {
        return GJKRecurse(ctx, v1, v0, iteration);
    }

    const t = closestT(v0.ab, v1.ab);
    const n = if (-1.0 < t and t < 1.0)
        vect.cpvperp(vect.cpvsub(v1.ab, v0.ab))
    else
        vect.cpvneg(lerpT(v0.ab, v1.ab, t));
    const p = Support(ctx, n);

    if (cpCheckPointGreater(p.ab, v0.ab, vect.cpvzero) and cpCheckPointGreater(v1.ab, p.ab, vect.cpvzero)) {
        return EPA(ctx, v0, p, v1);
    }

    if (cpCheckAxis(v0.ab, v1.ab, p.ab, n)) {
        return closestPointsNew(v0, v1);
    }

    if (closestDist(v0.ab, p.ab) < closestDist(p.ab, v1.ab)) {
        return GJKRecurse(ctx, v0, p, iteration + 1);
    }

    return GJKRecurse(ctx, p, v1, iteration + 1);
}

fn GJK(ctx: *const SupportContext, id: *types.cpCollisionID) ClosestPoints {
    var v0: MinkowskiPoint = undefined;
    var v1: MinkowskiPoint = undefined;

    if (id.* != 0) {
        v0 = minkowskiPointNew(shapePoint(ctx.shape1, @intCast((id.* >> 24) & 0xFF)), shapePoint(ctx.shape2, @intCast((id.* >> 16) & 0xFF)));
        v1 = minkowskiPointNew(shapePoint(ctx.shape1, @intCast((id.* >> 8) & 0xFF)), shapePoint(ctx.shape2, @intCast(id.* & 0xFF)));
    } else {
        var axis = vect.cpvperp(vect.cpvsub(bb.cpBBCenter(shapeBase(ctx.shape1).bbValue()), bb.cpBBCenter(shapeBase(ctx.shape2).bbValue())));
        if (vect.cpveql(axis, vect.cpvzero)) axis = vect.cpv(1.0, 0.0);
        v0 = Support(ctx, axis);
        v1 = Support(ctx, vect.cpvneg(axis));
    }

    const points = GJKRecurse(ctx, v0, v1, 1);
    id.* = points.id;
    return points;
}
const EdgePoint = struct {
    p: vect.cpVect,
    hash: types.cpHashValue,
};

const Edge = struct {
    a: EdgePoint,
    b: EdgePoint,
    r: types.cpFloat,
    n: vect.cpVect,
};

fn hashVertex(shape: *const shape_base.cpShape, index: usize) types.cpHashValue {
    const base_val: u64 = @intCast(@intFromPtr(shape));
    const mixed: u64 = (base_val >> 4) ^ (base_val << 7) ^ (@as(u64, index) * 1099511628211);
    return @intCast(mixed);
}

fn hashPair(a: types.cpHashValue, b: types.cpHashValue) types.cpHashValue {
    return @as(types.cpHashValue, (a * 31) ^ b);
}

fn supportEdgeForSegment(proxy: SegmentProxy, n: vect.cpVect) Edge {
    const shape_ptr = &proxy.shape.base;
    if (vect.cpvdot(proxy.normal, n) > 0.0) {
        return .{
            .a = .{ .p = proxy.a, .hash = hashVertex(shape_ptr, 0) },
            .b = .{ .p = proxy.b, .hash = hashVertex(shape_ptr, 1) },
            .r = proxy.radius,
            .n = proxy.normal,
        };
    }
    return .{
        .a = .{ .p = proxy.b, .hash = hashVertex(shape_ptr, 1) },
        .b = .{ .p = proxy.a, .hash = hashVertex(shape_ptr, 0) },
        .r = proxy.radius,
        .n = vect.cpvneg(proxy.normal),
    };
}

fn edgeNormal(a: vect.cpVect, b_: vect.cpVect) vect.cpVect {
    return vect.cpvnormalize(vect.cpvperp(vect.cpvsub(b_, a)));
}

fn supportEdgeForPoly(proxy: PolyProxy, n: vect.cpVect) Edge {
    const count = proxy.vertices.len;
    if (count == 0) {
        const p = proxy.position;
        return .{ .a = .{ .p = p, .hash = hashVertex(&proxy.shape.base, 0) }, .b = .{ .p = p, .hash = hashVertex(&proxy.shape.base, 0) }, .r = proxy.radius, .n = n };
    }

    var index: usize = 0;
    var max_dot = -types.CP_INFINITY;
    for (proxy.vertices, 0..) |_, i| {
        const world = polyVertexWorld(proxy, i);
        const dot = vect.cpvdot(world, n);
        if (dot > max_dot) {
            max_dot = dot;
            index = i;
        }
    }

    const prev = if (index == 0) count - 1 else index - 1;
    const next = (index + 1) % count;
    const current = polyVertexWorld(proxy, index);
    const prev_v = polyVertexWorld(proxy, prev);
    const next_v = polyVertexWorld(proxy, next);

    const prev_normal = edgeNormal(prev_v, current);
    const next_normal = edgeNormal(current, next_v);

    if (vect.cpvdot(n, next_normal) > vect.cpvdot(n, prev_normal)) {
        return .{
            .a = .{ .p = current, .hash = hashVertex(&proxy.shape.base, index) },
            .b = .{ .p = next_v, .hash = hashVertex(&proxy.shape.base, next) },
            .r = proxy.radius,
            .n = next_normal,
        };
    }

    return .{
        .a = .{ .p = prev_v, .hash = hashVertex(&proxy.shape.base, prev) },
        .b = .{ .p = current, .hash = hashVertex(&proxy.shape.base, index) },
        .r = proxy.radius,
        .n = prev_normal,
    };
}

fn contactPoints(edge1: Edge, edge2: Edge, points: ClosestPoints, result: *CollisionResult) void {
    const mindist = edge1.r + edge2.r;
    if (points.d > mindist) return;

    const n = points.n;
    result.normal = n;

    const d_e1_a = vect.cpvcross(edge1.a.p, n);
    const d_e1_b = vect.cpvcross(edge1.b.p, n);
    const d_e2_a = vect.cpvcross(edge2.a.p, n);
    const d_e2_b = vect.cpvcross(edge2.b.p, n);

    const e1_denom = 1.0 / (d_e1_b - d_e1_a + types.CPFLOAT_MIN);
    const e2_denom = 1.0 / (d_e2_b - d_e2_a + types.CPFLOAT_MIN);

    const p1a = vect.cpvadd(vect.cpvmult(n, edge1.r), vect.cpvlerp(edge1.a.p, edge1.b.p, types.cpfclamp01((d_e2_b - d_e1_a) * e1_denom)));
    const p2a = vect.cpvadd(vect.cpvmult(n, -edge2.r), vect.cpvlerp(edge2.a.p, edge2.b.p, types.cpfclamp01((d_e1_a - d_e2_a) * e2_denom)));
    const dist_a = vect.cpvdot(vect.cpvsub(p2a, p1a), n);
    if (dist_a <= 0.0) {
        result.addContact(.{ .point = p1a, .normal = n, .distance = dist_a, .hash = hashPair(edge1.a.hash, edge2.b.hash) });
    }

    const p1b = vect.cpvadd(vect.cpvmult(n, edge1.r), vect.cpvlerp(edge1.a.p, edge1.b.p, types.cpfclamp01((d_e2_a - d_e1_a) * e1_denom)));
    const p2b = vect.cpvadd(vect.cpvmult(n, -edge2.r), vect.cpvlerp(edge2.a.p, edge2.b.p, types.cpfclamp01((d_e1_b - d_e2_a) * e2_denom)));
    const dist_b = vect.cpvdot(vect.cpvsub(p2b, p1b), n);
    if (dist_b <= 0.0) {
        result.addContact(.{ .point = p1b, .normal = n, .distance = dist_b, .hash = hashPair(edge1.b.hash, edge2.a.hash) });
    }
}

fn closestPointOnSegment(p: vect.cpVect, a: vect.cpVect, b_: vect.cpVect) vect.cpVect {
    const ab = vect.cpvsub(b_, a);
    const ab_len_sq = vect.cpvdot(ab, ab);
    if (ab_len_sq == 0) return a;

    const t = types.cpfclamp(vect.cpvdot(vect.cpvsub(p, a), ab) / ab_len_sq, 0.0, 1.0);
    return vect.cpvadd(a, vect.cpvmult(ab, t));
}

fn circleToCircle(a: *const circle.cpCircleShape, b: *const circle.cpCircleShape) CollisionResult {
    var result = CollisionResult.empty();
    const da = vect.cpvadd(a.base.body.p, vect.cpvrotate(a.offset, a.base.body.rotationVector()));
    const db = vect.cpvadd(b.base.body.p, vect.cpvrotate(b.offset, b.base.body.rotationVector()));

    const delta = vect.cpvsub(db, da);
    const distance = vect.cpvlength(delta);
    const combined = a.radius + b.radius;

    if (distance <= combined) {
        const normal = if (distance == 0.0) vect.cpv(1.0, 0.0) else vect.cpvmult(delta, 1.0 / distance);
        const contact_point = vect.cpvadd(da, vect.cpvmult(normal, a.radius));
        result.addContact(.{ .point = contact_point, .normal = normal, .distance = distance - combined, .hash = 0 });
    }

    return result;
}

fn circleToSegment(circle_shape: *const circle.cpCircleShape, segment_shape: *const segment.cpSegmentShape) CollisionResult {
    var result = CollisionResult.empty();
    const circle_center = vect.cpvadd(circle_shape.base.body.p, vect.cpvrotate(circle_shape.offset, circle_shape.base.body.rotationVector()));
    const seg = buildSegmentProxy(segment_shape);

    const closest = closestPointOnSegment(circle_center, seg.a, seg.b);
    const delta = vect.cpvsub(circle_center, closest);
    const distance = vect.cpvlength(delta);
    const combined = circle_shape.radius + seg.radius;

    if (distance <= combined) {
        const normal = if (distance > 0.0) vect.cpvmult(delta, 1.0 / distance) else seg.normal;
        const contact_point = vect.cpvadd(closest, vect.cpvmult(normal, seg.radius));
        result.addContact(.{ .point = contact_point, .normal = normal, .distance = distance - combined, .hash = 0 });
    }

    return result;
}

fn circleToPoly(circle_shape: *const circle.cpCircleShape, poly_shape: *const poly.cpPolyShape, id: *types.cpCollisionID) CollisionResult {
    var result = CollisionResult.empty();
    const circle_proxy = SupportShape{ .circle = buildCircleProxy(circle_shape) };
    const poly_proxy = SupportShape{ .poly = buildPolyProxy(poly_shape) };
    var ctx = SupportContext{ .shape1 = circle_proxy, .shape2 = poly_proxy };
    const points = GJK(&ctx, id);
    result.id = points.id;

    if (points.d <= circle_proxy.circle.radius + poly_proxy.poly.radius) {
        const n = points.n;
        result.addContact(.{ .point = vect.cpvadd(points.a, vect.cpvmult(n, circle_proxy.circle.radius)), .normal = n, .distance = points.d - circle_proxy.circle.radius - poly_proxy.poly.radius, .hash = 0 });
    }

    return result;
}

fn segmentToSegment(seg_a: *const segment.cpSegmentShape, seg_b: *const segment.cpSegmentShape, id: *types.cpCollisionID) CollisionResult {
    var result = CollisionResult.empty();
    const proxy_a = SupportShape{ .segment = buildSegmentProxy(seg_a) };
    const proxy_b = SupportShape{ .segment = buildSegmentProxy(seg_b) };
    var ctx = SupportContext{ .shape1 = proxy_a, .shape2 = proxy_b };
    const points = GJK(&ctx, id);
    result.id = points.id;

    if (points.d <= proxy_a.segment.radius + proxy_b.segment.radius) {
        contactPoints(supportEdgeForSegment(proxy_a.segment, points.n), supportEdgeForSegment(proxy_b.segment, vect.cpvneg(points.n)), points, &result);
    }

    return result;
}

fn segmentToPoly(seg_shape: *const segment.cpSegmentShape, poly_shape: *const poly.cpPolyShape, id: *types.cpCollisionID) CollisionResult {
    var result = CollisionResult.empty();
    const proxy_a = SupportShape{ .segment = buildSegmentProxy(seg_shape) };
    const proxy_b = SupportShape{ .poly = buildPolyProxy(poly_shape) };
    var ctx = SupportContext{ .shape1 = proxy_a, .shape2 = proxy_b };
    const points = GJK(&ctx, id);
    result.id = points.id;

    if (points.d <= proxy_a.segment.radius + proxy_b.poly.radius) {
        contactPoints(supportEdgeForSegment(proxy_a.segment, points.n), supportEdgeForPoly(proxy_b.poly, vect.cpvneg(points.n)), points, &result);
    }

    return result;
}

fn polyToPoly(shape_a: *const poly.cpPolyShape, shape_b: *const poly.cpPolyShape, id: *types.cpCollisionID) CollisionResult {
    var result = CollisionResult.empty();
    const proxy_a = SupportShape{ .poly = buildPolyProxy(shape_a) };
    const proxy_b = SupportShape{ .poly = buildPolyProxy(shape_b) };
    var ctx = SupportContext{ .shape1 = proxy_a, .shape2 = proxy_b };
    const points = GJK(&ctx, id);
    result.id = points.id;

    if (points.d <= proxy_a.poly.radius + proxy_b.poly.radius) {
        contactPoints(supportEdgeForPoly(proxy_a.poly, points.n), supportEdgeForPoly(proxy_b.poly, vect.cpvneg(points.n)), points, &result);
    }

    return result;
}

fn collideWithId(a: *const shape_base.cpShape, b: *const shape_base.cpShape, id: *types.cpCollisionID) CollisionResult {
    return switch (a.shape_type) {
        .circle => switch (b.shape_type) {
            .circle => circleToCircle(@ptrCast(a), @ptrCast(b)),
            .segment => circleToSegment(@ptrCast(a), @ptrCast(b)),
            .poly => circleToPoly(@ptrCast(a), @ptrCast(b), id),
        },
        .segment => switch (b.shape_type) {
            .circle => circleToSegment(@ptrCast(b), @ptrCast(a)),
            .segment => segmentToSegment(@ptrCast(a), @ptrCast(b), id),
            .poly => segmentToPoly(@ptrCast(a), @ptrCast(b), id),
        },
        .poly => switch (b.shape_type) {
            .circle => circleToPoly(@ptrCast(b), @ptrCast(a), id),
            .segment => segmentToPoly(@ptrCast(b), @ptrCast(a), id),
            .poly => polyToPoly(@ptrCast(a), @ptrCast(b), id),
        },
    };
}

pub fn collide(a: *const shape_base.cpShape, b: *const shape_base.cpShape) CollisionResult {
    const key = pairKey(a, b);
    var id = cachedCollisionId(key);
    const result = collideWithId(a, b, &id);
    storeCollisionId(key, id);
    return result;
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

pub fn testCircleSegmentCollision() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(0.5, 0.0));

    var circle_a = circle.cpCircleShape.init(&body_a, 0.5, vect.cpvzero);
    var segment_b = segment.cpSegmentShape.init(&body_b, vect.cpv(-1.0, 0.0), vect.cpv(1.0, 0.0), 0.1);

    const contact = collide(&circle_a.base, &segment_b.base);
    try std.testing.expect(contact.contactCount() == 1);
    try std.testing.expect(contact.contacts.constSlice()[0].distance < 0.0);
}

pub fn testSegmentPolyCollision() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(-0.25, 0.0));
    body_b.setPosition(vect.cpv(0.0, 0.0));

    var segment_a = segment.cpSegmentShape.init(&body_a, vect.cpv(-0.5, -0.25), vect.cpv(0.5, -0.25), 0.05);
    const verts = [_]vect.cpVect{ vect.cpv(-0.5, -0.5), vect.cpv(0.5, -0.5), vect.cpv(0.5, 0.5), vect.cpv(-0.5, 0.5) };
    var poly_b = poly.cpPolyShape.init(&body_b, &verts, 0.0);

    const result = collide(&segment_a.base, &poly_b.base);
    try std.testing.expect(result.contactCount() >= 1);
}

pub fn testPolyPolyCollision() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    body_a.setPosition(vect.cpv(0.0, 0.0));
    body_b.setPosition(vect.cpv(0.5, 0.0));

    const verts = [_]vect.cpVect{ vect.cpv(-0.5, -0.5), vect.cpv(0.5, -0.5), vect.cpv(0.5, 0.5), vect.cpv(-0.5, 0.5) };
    var poly_a = poly.cpPolyShape.init(&body_a, &verts, 0.0);
    var poly_b = poly.cpPolyShape.init(&body_b, &verts, 0.0);

    const result = collide(&poly_a.base, &poly_b.base);
    try std.testing.expect(result.contactCount() >= 1);
    try std.testing.expect(result.contacts.constSlice()[0].distance <= 0.0);
}

comptime {
    std.testing.refAllDecls(@This());
}
