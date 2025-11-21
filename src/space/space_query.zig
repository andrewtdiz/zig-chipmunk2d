const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const shape_base = @import("../shape/shape_base.zig");
const circle = @import("../shape/circle.zig");
const segment = @import("../shape/segment.zig");
const poly = @import("../shape/poly.zig");
const collision = @import("../collision/collision.zig");

pub fn makeQueryApi(comptime Space: type) type {
    return struct {
        pub fn pointQuery(
            space_const: *const Space,
            point: vect.cpVect,
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, vect.cpVect, types.cpFloat, ?*anyopaque) void,
            data: ?*anyopaque,
        ) void {
            var space = @constCast(space_const);
            var ctx = PointContext{ .point = point, .filter = filter, .func = func, .data = data };
            const bounds = bb.cpBBNew(point.x, point.y, point.x, point.y);
            space.dynamic_index.query(bounds, pointCallback, &ctx);
            space.static_index.query(bounds, pointCallback, &ctx);
        }

        pub fn bbQuery(
            space_const: *const Space,
            bounds: bb.cpBB,
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, ?*anyopaque) void,
            data: ?*anyopaque,
        ) void {
            var space = @constCast(space_const);
            var ctx = BBContext{ .filter = filter, .func = func, .data = data };
            space.dynamic_index.query(bounds, bbCallback, &ctx);
            space.static_index.query(bounds, bbCallback, &ctx);
        }

        pub fn segmentQuery(
            space_const: *const Space,
            start: vect.cpVect,
            end: vect.cpVect,
            radius: types.cpFloat,
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, vect.cpVect, vect.cpVect, types.cpFloat, ?*anyopaque) void,
            data: ?*anyopaque,
        ) void {
            var space = @constCast(space_const);
            var ctx = SegmentContext{ .start = start, .end = end, .radius = radius, .filter = filter, .func = func, .data = data };
            const bounds = segmentBounds(start, end, radius);
            space.dynamic_index.query(bounds, segmentCallback, &ctx);
            space.static_index.query(bounds, segmentCallback, &ctx);
        }

        pub fn shapeQuery(
            space_const: *const Space,
            target: *shape_base.cpShape,
            func: fn (*shape_base.cpShape, collision.CollisionResult, ?*anyopaque) void,
            data: ?*anyopaque,
        ) void {
            var space = @constCast(space_const);
            var ctx = ShapeContext{ .target = target, .func = func, .cache = &space.collision_cache, .data = data };
            const bounds = target.bbValue();
            space.dynamic_index.query(bounds, shapeCallback, &ctx);
            space.static_index.query(bounds, shapeCallback, &ctx);
        }

        const PointContext = struct {
            point: vect.cpVect,
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, vect.cpVect, types.cpFloat, ?*anyopaque) void,
            data: ?*anyopaque,
        };

        const BBContext = struct {
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, ?*anyopaque) void,
            data: ?*anyopaque,
        };

        const SegmentContext = struct {
            start: vect.cpVect,
            end: vect.cpVect,
            radius: types.cpFloat,
            filter: shape_base.cpShapeFilter,
            func: fn (*shape_base.cpShape, vect.cpVect, vect.cpVect, types.cpFloat, ?*anyopaque) void,
            data: ?*anyopaque,
        };

        const ShapeContext = struct {
            target: *shape_base.cpShape,
            func: fn (*shape_base.cpShape, collision.CollisionResult, ?*anyopaque) void,
            cache: *collision.CollisionIdCache,
            data: ?*anyopaque,
        };

        fn pointCallback(object: *const anyopaque, _: bb.cpBB, ctx_ptr: ?*anyopaque) void {
            const ctx = @as(*PointContext, @ptrCast(ctx_ptr.?));
            const shape = @as(*shape_base.cpShape, @ptrCast(object));
            if (shape_base.cpShapeFilter.reject(shape.filter, ctx.filter)) return;
            const info = pointInfo(shape, ctx.point);
            if (info.distance <= 0.0) {
                ctx.func(shape, ctx.point, info.distance, ctx.data);
            }
        }

        fn bbCallback(object: *const anyopaque, object_bounds: bb.cpBB, ctx_ptr: ?*anyopaque) void {
            const ctx = @as(*BBContext, @ptrCast(ctx_ptr.?));
            const shape = @as(*shape_base.cpShape, @ptrCast(object));
            if (shape_base.cpShapeFilter.reject(shape.filter, ctx.filter)) return;
            if (bb.cpBBIntersects(shape.bbValue(), object_bounds)) {
                ctx.func(shape, ctx.data);
            }
        }

        fn segmentCallback(object: *const anyopaque, _: bb.cpBB, ctx_ptr: ?*anyopaque) void {
            const ctx = @as(*SegmentContext, @ptrCast(ctx_ptr.?));
            const shape = @as(*shape_base.cpShape, @ptrCast(object));
            if (shape_base.cpShapeFilter.reject(shape.filter, ctx.filter)) return;
            if (segmentHit(shape, ctx.start, ctx.end, ctx.radius)) |hit| {
                ctx.func(shape, hit.point, hit.normal, hit.alpha, ctx.data);
            }
        }

        fn shapeCallback(object: *const anyopaque, _: bb.cpBB, ctx_ptr: ?*anyopaque) void {
            const ctx = @as(*ShapeContext, @ptrCast(ctx_ptr.?));
            const shape = @as(*shape_base.cpShape, @ptrCast(object));
            if (shape == ctx.target) return;
            const result = collision.collide(ctx.cache, ctx.target, shape);
            if (result.contactCount() > 0) {
                ctx.func(shape, result, ctx.data);
            }
        }

        const SegmentHit = struct {
            point: vect.cpVect,
            normal: vect.cpVect,
            alpha: types.cpFloat,
        };

        const PointInfo = struct {
            distance: types.cpFloat,
            nearest: vect.cpVect,
            gradient: vect.cpVect,
        };

        fn pointInfo(shape: *const shape_base.cpShape, point: vect.cpVect) PointInfo {
            return switch (shape.shape_type) {
                .circle => circlePointInfo(asCircle(shape), point),
                .segment => segmentPointInfo(asSegment(shape), point),
                .poly => polyPointInfo(asPoly(shape), point),
            };
        }

        fn circlePointInfo(shape: *const circle.cpCircleShape, point: vect.cpVect) PointInfo {
            const rot = shape.base.body.rotationVector();
            const center = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.offset, rot));
            const delta = vect.cpvsub(point, center);
            const dist = vect.cpvlength(delta);
            const gradient = if (dist > 0.0) vect.cpvmult(delta, 1.0 / dist) else vect.cpv(0.0, 1.0);
            const nearest = vect.cpvadd(center, vect.cpvmult(gradient, shape.radius));
            return .{ .distance = dist - shape.radius, .nearest = nearest, .gradient = gradient };
        }

        fn segmentPointInfo(shape: *const segment.cpSegmentShape, point: vect.cpVect) PointInfo {
            const rot = shape.base.body.rotationVector();
            const a = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.a, rot));
            const b_ = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.b, rot));
            const closest = closestPoint(point, a, b_);
            const delta = vect.cpvsub(point, closest);
            const dist = vect.cpvlength(delta);
            const fallback = vect.cpvrotate(shape.normal, rot);
            const gradient = if (dist > 0.0) vect.cpvmult(delta, 1.0 / dist) else fallback;
            const nearest = vect.cpvadd(closest, vect.cpvmult(gradient, shape.radius));
            return .{ .distance = dist - shape.radius, .nearest = nearest, .gradient = gradient };
        }

        fn polyPointInfo(shape: *const poly.cpPolyShape, point: vect.cpVect) PointInfo {
            const count = shape.vertices.len;
            if (count == 0) {
                return .{ .distance = types.CP_INFINITY, .nearest = point, .gradient = vect.cpv(0.0, 1.0) };
            }
            const rot = shape.base.body.rotationVector();
            const offset = shape.base.body.p;
            var inside = true;
            var min_dist = types.CP_INFINITY;
            var best_point = vect.cpvzero;
            var best_grad = vect.cpv(0.0, 1.0);

            var prev = vect.cpvadd(offset, vect.cpvrotate(shape.vertices[count - 1], rot));
            var i: usize = 0;
            while (i < count) : (i += 1) {
                const current = vect.cpvadd(offset, vect.cpvrotate(shape.vertices[i], rot));
                const edge = vect.cpvsub(current, prev);
                if (vect.cpvlengthsq(edge) == 0.0) {
                    prev = current;
                    continue;
                }
                const normal = vect.cpvperp(edge);
                if (vect.cpvdot(normal, vect.cpvsub(point, prev)) > 0.0) inside = false;

                const projected = closestPoint(point, prev, current);
                const delta = vect.cpvsub(point, projected);
                const dist = vect.cpvlength(delta);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_point = projected;
                    const fallback = vect.cpvnormalize(normal);
                    best_grad = if (dist > 0.0) vect.cpvmult(delta, 1.0 / dist) else fallback;
                }

                prev = current;
            }

            if (min_dist == types.CP_INFINITY) {
                return .{ .distance = types.CP_INFINITY, .nearest = point, .gradient = vect.cpv(0.0, 1.0) };
            }

            const signed_dist = if (inside) -min_dist else min_dist;
            const nearest = vect.cpvsub(best_point, vect.cpvmult(best_grad, shape.radius));
            return .{ .distance = signed_dist - shape.radius, .nearest = nearest, .gradient = best_grad };
        }

        fn segmentHit(shape: *const shape_base.cpShape, start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat) ?SegmentHit {
            const info = pointInfo(shape, start);
            if (info.distance <= radius) {
                const delta = vect.cpvsub(start, info.nearest);
                const len = vect.cpvlength(delta);
                const normal = if (len > 0.0) vect.cpvmult(delta, 1.0 / len) else info.gradient;
                return SegmentHit{ .point = info.nearest, .normal = normal, .alpha = 0.0 };
            }

            return switch (shape.shape_type) {
                .circle => circleSegmentHit(asCircle(shape), start, end, radius),
                .segment => segmentSegmentHit(asSegment(shape), start, end, radius),
                .poly => polySegmentHit(asPoly(shape), start, end, radius),
            };
        }

        fn circleSegmentHit(shape: *const circle.cpCircleShape, start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat) ?SegmentHit {
            const rot = shape.base.body.rotationVector();
            const center = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.offset, rot));
            return circleSegmentQuery(center, shape.radius, start, end, radius);
        }

        fn segmentSegmentHit(shape: *const segment.cpSegmentShape, start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat) ?SegmentHit {
            const rot = shape.base.body.rotationVector();
            const a_world = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.a, rot));
            const b_world = vect.cpvadd(shape.base.body.p, vect.cpvrotate(shape.b, rot));
            const normal = vect.cpvrotate(shape.normal, rot);
            const delta = vect.cpvsub(end, start);
            const d = vect.cpvdot(vect.cpvsub(a_world, start), normal);
            const combined = shape.radius + radius;
            const flipped = if (d > 0.0) vect.cpvneg(normal) else normal;
            const seg_offset = vect.cpvsub(vect.cpvmult(flipped, combined), start);
            const seg_a = vect.cpvadd(a_world, seg_offset);
            const seg_b = vect.cpvadd(b_world, seg_offset);

            if (vect.cpvcross(delta, seg_a) * vect.cpvcross(delta, seg_b) <= 0.0) {
                const d_offset = d + (if (d > 0.0) -combined else combined);
                const ad = -d_offset;
                const bd = vect.cpvdot(delta, normal) - d_offset;
                const denom = ad - bd;
                if (ad * bd < 0.0 and denom != 0.0) {
                    const t = ad / denom;
                    if (t >= 0.0 and t <= 1.0) {
                        const point = vect.cpvsub(vect.cpvlerp(start, end, t), vect.cpvmult(flipped, radius));
                        return SegmentHit{ .point = point, .normal = flipped, .alpha = t };
                    }
                }
            } else if (combined > 0.0) {
                var best: ?SegmentHit = null;
                var best_alpha: types.cpFloat = 1.0;
                if (circleSegmentQuery(a_world, shape.radius, start, end, radius)) |hit| {
                    best = hit;
                    best_alpha = hit.alpha;
                }
                if (circleSegmentQuery(b_world, shape.radius, start, end, radius)) |hit| {
                    if (hit.alpha < best_alpha) {
                        best = hit;
                        best_alpha = hit.alpha;
                    }
                }
                return best;
            }

            return null;
        }

        fn polySegmentHit(shape: *const poly.cpPolyShape, start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat) ?SegmentHit {
            const count = shape.vertices.len;
            if (count == 0) return null;
            const rot = shape.base.body.rotationVector();
            const offset = shape.base.body.p;
            const rsum = shape.radius + radius;
            var best: ?SegmentHit = null;
            var best_alpha: types.cpFloat = 1.0;

            var prev = vect.cpvadd(offset, vect.cpvrotate(shape.vertices[count - 1], rot));
            var i: usize = 0;
            while (i < count) : (i += 1) {
                const current = vect.cpvadd(offset, vect.cpvrotate(shape.vertices[i], rot));
                const edge = vect.cpvsub(current, prev);
                const edge_len_sq = vect.cpvlengthsq(edge);
                if (edge_len_sq == 0.0) {
                    prev = current;
                    continue;
                }
                const normal = vect.cpvnormalize(vect.cpvperp(edge));
                const an = vect.cpvdot(start, normal);
                const plane_distance = vect.cpvdot(current, normal);
                const d = an - plane_distance - rsum;
                if (d < 0.0) {
                    prev = current;
                    continue;
                }

                const bn = vect.cpvdot(end, normal);
                const denom = an - bn;
                if (denom == 0.0) {
                    prev = current;
                    continue;
                }

                const t = d / denom;
                if (t < 0.0 or t > 1.0) {
                    prev = current;
                    continue;
                }

                const point = vect.cpvlerp(start, end, t);
                const dt = vect.cpvcross(normal, point);
                const dt_min = types.cpfmin(vect.cpvcross(normal, prev), vect.cpvcross(normal, current));
                const dt_max = types.cpfmax(vect.cpvcross(normal, prev), vect.cpvcross(normal, current));
                if (dt_min <= dt and dt <= dt_max and t < best_alpha) {
                    best_alpha = t;
                    best = SegmentHit{ .point = vect.cpvsub(point, vect.cpvmult(normal, radius)), .normal = normal, .alpha = t };
                }

                prev = current;
            }

            if (rsum > 0.0) {
                var j: usize = 0;
                while (j < count) : (j += 1) {
                    const vertex = vect.cpvadd(offset, vect.cpvrotate(shape.vertices[j], rot));
                    if (circleSegmentQuery(vertex, shape.radius, start, end, radius)) |hit| {
                        if (hit.alpha < best_alpha) {
                            best_alpha = hit.alpha;
                            best = hit;
                        }
                    }
                }
            }

            return best;
        }

        fn circleSegmentQuery(center: vect.cpVect, radius_a: types.cpFloat, start: vect.cpVect, end: vect.cpVect, radius_b: types.cpFloat) ?SegmentHit {
            const da = vect.cpvsub(start, center);
            const db = vect.cpvsub(end, center);
            const delta = vect.cpvsub(end, start);
            const qa = vect.cpvdot(delta, delta);
            if (qa <= types.CPFLOAT_MIN) return null;
            const qb = vect.cpvdot(da, delta);
            const da_len_sq = vect.cpvdot(da, da);
            const rsum = radius_a + radius_b;
            const det = qb * qb - qa * (da_len_sq - rsum * rsum);
            if (det < 0.0) return null;
            const sqrt_det = types.cpfsqrt(det);
            const t = (-qb - sqrt_det) / qa;
            if (t < 0.0 or t > 1.0) return null;

            const contact = vect.cpvlerp(da, db, t);
            const length = vect.cpvlength(contact);
            if (length <= 0.0) return null;
            const normal = vect.cpvmult(contact, 1.0 / length);
            const point = vect.cpvsub(vect.cpvlerp(start, end, t), vect.cpvmult(normal, radius_b));
            return SegmentHit{ .point = point, .normal = normal, .alpha = t };
        }

        fn segmentBounds(start: vect.cpVect, end: vect.cpVect, radius: types.cpFloat) bb.cpBB {
            return bb.cpBBNew(
                types.cpfmin(start.x, end.x) - radius,
                types.cpfmin(start.y, end.y) - radius,
                types.cpfmax(start.x, end.x) + radius,
                types.cpfmax(start.y, end.y) + radius,
            );
        }

        fn closestPoint(p: vect.cpVect, a: vect.cpVect, b_: vect.cpVect) vect.cpVect {
            const ab = vect.cpvsub(b_, a);
            const denom = vect.cpvdot(ab, ab);
            if (denom == 0.0) return a;
            const t = types.cpfclamp(vect.cpvdot(vect.cpvsub(p, a), ab) / denom, 0.0, 1.0);
            return vect.cpvadd(a, vect.cpvmult(ab, t));
        }

        fn asCircle(shape: *const shape_base.cpShape) *circle.cpCircleShape {
            return @as(*circle.cpCircleShape, @ptrCast(shape));
        }

        fn asSegment(shape: *const shape_base.cpShape) *segment.cpSegmentShape {
            return @as(*segment.cpSegmentShape, @ptrCast(shape));
        }

        fn asPoly(shape: *const shape_base.cpShape) *poly.cpPolyShape {
            return @as(*poly.cpPolyShape, @ptrCast(shape));
        }
    };
}
