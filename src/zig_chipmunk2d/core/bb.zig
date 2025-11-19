const std = @import("std");
const types = @import("types.zig");
const vect = @import("vect.zig");

pub const cpBB = struct {
    l: types.cpFloat,
    b: types.cpFloat,
    r: types.cpFloat,
    t: types.cpFloat,
};

pub inline fn cpBBNew(l: types.cpFloat, b: types.cpFloat, r: types.cpFloat, t: types.cpFloat) cpBB {
    return .{ .l = l, .b = b, .r = r, .t = t };
}

pub inline fn cpBBNewForExtents(center: vect.cpVect, half_width: types.cpFloat, half_height: types.cpFloat) cpBB {
    return cpBBNew(center.x - half_width, center.y - half_height, center.x + half_width, center.y + half_height);
}

pub inline fn cpBBNewForCircle(position: vect.cpVect, radius: types.cpFloat) cpBB {
    return cpBBNewForExtents(position, radius, radius);
}

pub inline fn cpBBIntersects(a: cpBB, b: cpBB) types.cpBool {
    return a.l <= b.r and b.l <= a.r and a.b <= b.t and b.b <= a.t;
}

pub inline fn cpBBContainsBB(bb: cpBB, other: cpBB) types.cpBool {
    return bb.l <= other.l and bb.r >= other.r and bb.b <= other.b and bb.t >= other.t;
}

pub inline fn cpBBContainsVect(bb: cpBB, v: vect.cpVect) types.cpBool {
    return bb.l <= v.x and bb.r >= v.x and bb.b <= v.y and bb.t >= v.y;
}

pub inline fn cpBBMerge(a: cpBB, b: cpBB) cpBB {
    return cpBBNew(types.cpfmin(a.l, b.l), types.cpfmin(a.b, b.b), types.cpfmax(a.r, b.r), types.cpfmax(a.t, b.t));
}

pub inline fn cpBBExpand(bb: cpBB, v: vect.cpVect) cpBB {
    return cpBBNew(types.cpfmin(bb.l, v.x), types.cpfmin(bb.b, v.y), types.cpfmax(bb.r, v.x), types.cpfmax(bb.t, v.y));
}

pub inline fn cpBBCenter(bb: cpBB) vect.cpVect {
    return vect.cpvlerp(vect.cpv(bb.l, bb.b), vect.cpv(bb.r, bb.t), 0.5);
}

pub inline fn cpBBArea(bb: cpBB) types.cpFloat {
    return (bb.r - bb.l) * (bb.t - bb.b);
}

pub inline fn cpBBMergedArea(a: cpBB, b: cpBB) types.cpFloat {
    return (types.cpfmax(a.r, b.r) - types.cpfmin(a.l, b.l)) * (types.cpfmax(a.t, b.t) - types.cpfmin(a.b, b.b));
}

pub inline fn cpBBSegmentQuery(bb: cpBB, a: vect.cpVect, b: vect.cpVect) types.cpFloat {
    const delta = vect.cpvsub(b, a);
    var tmin = -types.CP_INFINITY;
    var tmax = types.CP_INFINITY;

    if (delta.x == 0.0) {
        if (a.x < bb.l or bb.r < a.x) return types.CP_INFINITY;
    } else {
        const t1 = (bb.l - a.x) / delta.x;
        const t2 = (bb.r - a.x) / delta.x;
        tmin = types.cpfmax(tmin, types.cpfmin(t1, t2));
        tmax = types.cpfmin(tmax, types.cpfmax(t1, t2));
    }

    if (delta.y == 0.0) {
        if (a.y < bb.b or bb.t < a.y) return types.CP_INFINITY;
    } else {
        const t1 = (bb.b - a.y) / delta.y;
        const t2 = (bb.t - a.y) / delta.y;
        tmin = types.cpfmax(tmin, types.cpfmin(t1, t2));
        tmax = types.cpfmin(tmax, types.cpfmax(t1, t2));
    }

    if (tmin <= tmax and 0.0 <= tmax and tmin <= 1.0) {
        return types.cpfmax(tmin, 0.0);
    }

    return types.CP_INFINITY;
}

pub inline fn cpBBIntersectsSegment(bb: cpBB, a: vect.cpVect, b: vect.cpVect) types.cpBool {
    return cpBBSegmentQuery(bb, a, b) != types.CP_INFINITY;
}

pub inline fn cpBBClampVect(bb: cpBB, v: vect.cpVect) vect.cpVect {
    return vect.cpv(types.cpfclamp(v.x, bb.l, bb.r), types.cpfclamp(v.y, bb.b, bb.t));
}

pub inline fn cpBBWrapVect(bb: cpBB, v: vect.cpVect) vect.cpVect {
    const dx = types.cpfabs(bb.r - bb.l);
    const modx = types.cpfmod(v.x - bb.l, dx);
    const x = if (modx > 0.0) modx else modx + dx;

    const dy = types.cpfabs(bb.t - bb.b);
    const mody = types.cpfmod(v.y - bb.b, dy);
    const y = if (mody > 0.0) mody else mody + dy;

    return vect.cpv(x + bb.l, y + bb.b);
}

pub inline fn cpBBOffset(bb: cpBB, v: vect.cpVect) cpBB {
    return cpBBNew(bb.l + v.x, bb.b + v.y, bb.r + v.x, bb.t + v.y);
}

test "cpBB helpers" {
    const bb = cpBBNew(0.0, 0.0, 10.0, 10.0);
    try std.testing.expect(cpBBIntersects(bb, cpBBNew(5.0, 5.0, 12.0, 12.0)));
    try std.testing.expect(cpBBContainsVect(bb, vect.cpv(2.0, 3.0)));
    try std.testing.expectEqual(@as(types.cpFloat, 100.0), cpBBArea(bb));
    try std.testing.expect(cpBBIntersectsSegment(bb, vect.cpv(-5.0, 5.0), vect.cpv(5.0, 5.0)));
}
