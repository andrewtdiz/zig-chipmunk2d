const std = @import("std");
const types = @import("types.zig");
const vect = @import("vect.zig");
const bb = @import("bb.zig");

pub const cpTransform = struct {
    a: types.cpFloat,
    b: types.cpFloat,
    c: types.cpFloat,
    d: types.cpFloat,
    tx: types.cpFloat,
    ty: types.cpFloat,
};

pub const cpTransformIdentity = cpTransform{ .a = 1.0, .b = 0.0, .c = 0.0, .d = 1.0, .tx = 0.0, .ty = 0.0 };

pub inline fn cpTransformNew(a: types.cpFloat, b_: types.cpFloat, c_: types.cpFloat, d_: types.cpFloat, tx: types.cpFloat, ty: types.cpFloat) cpTransform {
    return .{ .a = a, .b = b_, .c = c_, .d = d_, .tx = tx, .ty = ty };
}

pub inline fn cpTransformNewTranspose(a: types.cpFloat, c_: types.cpFloat, tx: types.cpFloat, b_: types.cpFloat, d_: types.cpFloat, ty: types.cpFloat) cpTransform {
    return cpTransformNew(a, b_, c_, d_, tx, ty);
}

pub inline fn cpTransformInverse(t: cpTransform) cpTransform {
    const inv_det = 1.0 / (t.a * t.d - t.c * t.b);
    return cpTransformNewTranspose(
        t.d * inv_det,
        -t.c * inv_det,
        (t.c * t.ty - t.tx * t.d) * inv_det,
        -t.b * inv_det,
        t.a * inv_det,
        (t.tx * t.b - t.a * t.ty) * inv_det,
    );
}

pub inline fn cpTransformMult(t1: cpTransform, t2: cpTransform) cpTransform {
    return cpTransformNewTranspose(
        t1.a * t2.a + t1.c * t2.b,
        t1.a * t2.c + t1.c * t2.d,
        t1.a * t2.tx + t1.c * t2.ty + t1.tx,
        t1.b * t2.a + t1.d * t2.b,
        t1.b * t2.c + t1.d * t2.d,
        t1.b * t2.tx + t1.d * t2.ty + t1.ty,
    );
}

pub inline fn cpTransformPoint(t: cpTransform, p: vect.cpVect) vect.cpVect {
    return vect.cpv(t.a * p.x + t.c * p.y + t.tx, t.b * p.x + t.d * p.y + t.ty);
}

pub inline fn cpTransformVect(t: cpTransform, v: vect.cpVect) vect.cpVect {
    return vect.cpv(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y);
}

pub inline fn cpTransformbBB(t: cpTransform, bounds: bb.cpBB) bb.cpBB {
    const center = bb.cpBBCenter(bounds);
    const half_width = (bounds.r - bounds.l) * 0.5;
    const half_height = (bounds.t - bounds.b) * 0.5;

    const a = t.a * half_width;
    const b_ = t.c * half_height;
    const d_ = t.b * half_width;
    const e = t.d * half_height;
    const hw_max = types.cpfmax(types.cpfabs(a + b_), types.cpfabs(a - b_));
    const hh_max = types.cpfmax(types.cpfabs(d_ + e), types.cpfabs(d_ - e));
    return bb.cpBBNewForExtents(cpTransformPoint(t, center), hw_max, hh_max);
}

pub inline fn cpTransformTranslate(translate: vect.cpVect) cpTransform {
    return cpTransformNewTranspose(1.0, 0.0, translate.x, 0.0, 1.0, translate.y);
}

pub inline fn cpTransformScale(scale_x: types.cpFloat, scale_y: types.cpFloat) cpTransform {
    return cpTransformNewTranspose(scale_x, 0.0, 0.0, 0.0, scale_y, 0.0);
}

pub inline fn cpTransformRotate(radians: types.cpFloat) cpTransform {
    const rot = vect.cpvforangle(radians);
    return cpTransformNewTranspose(rot.x, -rot.y, 0.0, rot.y, rot.x, 0.0);
}

pub inline fn cpTransformRigid(translate: vect.cpVect, radians: types.cpFloat) cpTransform {
    const rot = vect.cpvforangle(radians);
    return cpTransformNewTranspose(rot.x, -rot.y, translate.x, rot.y, rot.x, translate.y);
}

pub inline fn cpTransformRigidInverse(t: cpTransform) cpTransform {
    return cpTransformNewTranspose(
        t.d,
        -t.c,
        (t.c * t.ty - t.tx * t.d),
        -t.b,
        t.a,
        (t.tx * t.b - t.a * t.ty),
    );
}

pub inline fn cpTransformWrap(outer: cpTransform, inner: cpTransform) cpTransform {
    return cpTransformMult(cpTransformInverse(outer), cpTransformMult(inner, outer));
}

pub inline fn cpTransformWrapInverse(outer: cpTransform, inner: cpTransform) cpTransform {
    return cpTransformMult(outer, cpTransformMult(inner, cpTransformInverse(outer)));
}

pub inline fn cpTransformOrtho(bounds: bb.cpBB) cpTransform {
    return cpTransformNewTranspose(
        2.0 / (bounds.r - bounds.l),
        0.0,
        -(bounds.r + bounds.l) / (bounds.r - bounds.l),
        0.0,
        2.0 / (bounds.t - bounds.b),
        -(bounds.t + bounds.b) / (bounds.t - bounds.b),
    );
}

pub inline fn cpTransformBoneScale(v0: vect.cpVect, v1: vect.cpVect) cpTransform {
    const d = vect.cpvsub(v1, v0);
    return cpTransformNewTranspose(d.x, -d.y, v0.x, d.y, d.x, v0.y);
}

pub inline fn cpTransformAxialScale(axis: vect.cpVect, pivot: vect.cpVect, scale: types.cpFloat) cpTransform {
    const A = axis.x * axis.y * (scale - 1.0);
    const B = vect.cpvdot(axis, pivot) * (1.0 - scale);

    return cpTransformNewTranspose(
        scale * axis.x * axis.x + axis.y * axis.y,
        A,
        axis.x * B,
        A,
        axis.x * axis.x + scale * axis.y * axis.y,
        axis.y * B,
    );
}

test "cpTransform basics" {
    const translate = cpTransformTranslate(vect.cpv(2.0, 3.0));
    const rotated = cpTransformRotate(types.CP_PI / 2.0);
    const combined = cpTransformMult(translate, rotated);

    const point = cpTransformPoint(combined, vect.cpv(1.0, 0.0));
    try std.testing.expectApproxEqAbs(2.0, point.x, 1e-6);
    try std.testing.expectApproxEqAbs(4.0, point.y, 1e-6);
}
