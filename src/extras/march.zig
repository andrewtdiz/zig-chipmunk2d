const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");

pub const cpMarchSegmentFunc = fn (vect.cpVect, vect.cpVect, ?*anyopaque) void;
pub const cpMarchSampleFunc = fn (vect.cpVect, ?*anyopaque) types.cpFloat;

fn seg(v0: vect.cpVect, v1: vect.cpVect, f: cpMarchSegmentFunc, data: ?*anyopaque) void {
    if (!vect.cpveql(v0, v1)) f(v1, v0, data);
}

fn midlerp(x0: types.cpFloat, x1: types.cpFloat, s0: types.cpFloat, s1: types.cpFloat, t: types.cpFloat) types.cpFloat {
    return types.cpflerp(x0, x1, (t - s0) / (s1 - s0));
}

fn marchCells(
    allocator: std.mem.Allocator,
    bounds: bb.cpBB,
    x_samples: usize,
    y_samples: usize,
    t: types.cpFloat,
    segment: cpMarchSegmentFunc,
    segment_data: ?*anyopaque,
    sample: cpMarchSampleFunc,
    sample_data: ?*anyopaque,
    cell: fn (types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, types.cpFloat, cpMarchSegmentFunc, ?*anyopaque) void,
) void {
    if (x_samples < 2 or y_samples < 2) return;

    const x_denom: types.cpFloat = 1.0 / @as(types.cpFloat, @floatFromInt(x_samples - 1));
    const y_denom: types.cpFloat = 1.0 / @as(types.cpFloat, @floatFromInt(y_samples - 1));

    var buffer = std.ArrayList(types.cpFloat).init(allocator);
    defer buffer.deinit();
    buffer.ensureTotalCapacity(x_samples) catch {};

    var i: usize = 0;
    while (i < x_samples) : (i += 1) {
        const x = types.cpflerp(bounds.l, bounds.r, @as(types.cpFloat, @floatFromInt(i)) * x_denom);
        buffer.append(sample(vect.cpv(x, bounds.b), sample_data)) catch {};
    }

    var j: usize = 0;
    while (j < y_samples - 1) : (j += 1) {
        const y0 = types.cpflerp(bounds.b, bounds.t, @as(types.cpFloat, @floatFromInt(j)) * y_denom);
        const y1 = types.cpflerp(bounds.b, bounds.t, @as(types.cpFloat, @floatFromInt(j + 1)) * y_denom);

        var a: types.cpFloat = undefined;
        var b_val: types.cpFloat = buffer.items[0];
        var c: types.cpFloat = undefined;
        var d: types.cpFloat = sample(vect.cpv(bounds.l, y1), sample_data);
        buffer.items[0] = d;

        var xi: usize = 0;
        while (xi < x_samples - 1) : (xi += 1) {
            const x0 = types.cpflerp(bounds.l, bounds.r, @as(types.cpFloat, @floatFromInt(xi)) * x_denom);
            const x1 = types.cpflerp(bounds.l, bounds.r, @as(types.cpFloat, @floatFromInt(xi + 1)) * x_denom);

            a = b_val;
            b_val = buffer.items[xi + 1];
            c = d;
            d = sample(vect.cpv(x1, y1), sample_data);
            buffer.items[xi + 1] = d;

            cell(t, a, b_val, c, d, x0, x1, y0, y1, segment, segment_data);
        }
    }
}

fn cellSoft(
    t: types.cpFloat,
    a: types.cpFloat,
    b_val: types.cpFloat,
    c: types.cpFloat,
    d: types.cpFloat,
    x0: types.cpFloat,
    x1: types.cpFloat,
    y0: types.cpFloat,
    y1: types.cpFloat,
    segment: cpMarchSegmentFunc,
    segment_data: ?*anyopaque,
) void {
    switch ((a > t) << 0 | (b_val > t) << 1 | (c > t) << 2 | (d > t) << 3) {
        0x1 => seg(vect.cpv(x0, midlerp(y0, y1, a, c, t)), vect.cpv(midlerp(x0, x1, a, b_val, t), y0), segment, segment_data),
        0x2 => seg(vect.cpv(midlerp(x0, x1, a, b_val, t), y0), vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), segment, segment_data),
        0x3 => seg(vect.cpv(x0, midlerp(y0, y1, a, c, t)), vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), segment, segment_data),
        0x4 => seg(vect.cpv(midlerp(x0, x1, c, d, t), y1), vect.cpv(x0, midlerp(y0, y1, a, c, t)), segment, segment_data),
        0x5 => seg(vect.cpv(midlerp(x0, x1, c, d, t), y1), vect.cpv(midlerp(x0, x1, a, b_val, t), y0), segment, segment_data),
        0x6 => {
            seg(vect.cpv(midlerp(x0, x1, a, b_val, t), y0), vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), segment, segment_data);
            seg(vect.cpv(midlerp(x0, x1, c, d, t), y1), vect.cpv(x0, midlerp(y0, y1, a, c, t)), segment, segment_data);
        },
        0x7 => seg(vect.cpv(midlerp(x0, x1, c, d, t), y1), vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), segment, segment_data),
        0x8 => seg(vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), vect.cpv(midlerp(x0, x1, c, d, t), y1), segment, segment_data),
        0x9 => {
            seg(vect.cpv(x0, midlerp(y0, y1, a, c, t)), vect.cpv(midlerp(x0, x1, a, b_val, t), y0), segment, segment_data);
            seg(vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), vect.cpv(midlerp(x0, x1, c, d, t), y1), segment, segment_data);
        },
        0xA => seg(vect.cpv(midlerp(x0, x1, a, b_val, t), y0), vect.cpv(midlerp(x0, x1, c, d, t), y1), segment, segment_data),
        0xB => seg(vect.cpv(x0, midlerp(y0, y1, a, c, t)), vect.cpv(midlerp(x0, x1, c, d, t), y1), segment, segment_data),
        0xC => seg(vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), vect.cpv(x0, midlerp(y0, y1, a, c, t)), segment, segment_data),
        0xD => seg(vect.cpv(x1, midlerp(y0, y1, b_val, d, t)), vect.cpv(midlerp(x0, x1, a, b_val, t), y0), segment, segment_data),
        0xE => seg(vect.cpv(midlerp(x0, x1, a, b_val, t), y0), vect.cpv(x0, midlerp(y0, y1, a, c, t)), segment, segment_data),
        else => {},
    }
}

fn segs(a: vect.cpVect, b: vect.cpVect, c: vect.cpVect, f: cpMarchSegmentFunc, data: ?*anyopaque) void {
    seg(b, c, f, data);
    seg(a, b, f, data);
}

fn cellHard(
    t: types.cpFloat,
    a: types.cpFloat,
    b_val: types.cpFloat,
    c: types.cpFloat,
    d: types.cpFloat,
    x0: types.cpFloat,
    x1: types.cpFloat,
    y0: types.cpFloat,
    y1: types.cpFloat,
    segment: cpMarchSegmentFunc,
    segment_data: ?*anyopaque,
) void {
    const xm = types.cpflerp(x0, x1, 0.5);
    const ym = types.cpflerp(y0, y1, 0.5);

    switch ((a > t) << 0 | (b_val > t) << 1 | (c > t) << 2 | (d > t) << 3) {
        0x1 => segs(vect.cpv(x0, ym), vect.cpv(xm, ym), vect.cpv(xm, y0), segment, segment_data),
        0x2 => segs(vect.cpv(xm, y0), vect.cpv(xm, ym), vect.cpv(x1, ym), segment, segment_data),
        0x3 => seg(vect.cpv(x0, ym), vect.cpv(x1, ym), segment, segment_data),
        0x4 => segs(vect.cpv(xm, y1), vect.cpv(xm, ym), vect.cpv(x0, ym), segment, segment_data),
        0x5 => seg(vect.cpv(xm, y1), vect.cpv(xm, y0), segment, segment_data),
        0x6 => {
            segs(vect.cpv(xm, y0), vect.cpv(xm, ym), vect.cpv(x0, ym), segment, segment_data);
            segs(vect.cpv(xm, y1), vect.cpv(xm, ym), vect.cpv(x1, ym), segment, segment_data);
        },
        0x7 => segs(vect.cpv(xm, y1), vect.cpv(xm, ym), vect.cpv(x1, ym), segment, segment_data),
        0x8 => segs(vect.cpv(x1, ym), vect.cpv(xm, ym), vect.cpv(xm, y1), segment, segment_data),
        0x9 => {
            segs(vect.cpv(x1, ym), vect.cpv(xm, ym), vect.cpv(xm, y0), segment, segment_data);
            segs(vect.cpv(x0, ym), vect.cpv(xm, ym), vect.cpv(xm, y1), segment, segment_data);
        },
        0xA => seg(vect.cpv(xm, y0), vect.cpv(xm, y1), segment, segment_data),
        0xB => segs(vect.cpv(x0, ym), vect.cpv(xm, ym), vect.cpv(xm, y1), segment, segment_data),
        0xC => seg(vect.cpv(x1, ym), vect.cpv(x0, ym), segment, segment_data),
        0xD => segs(vect.cpv(x1, ym), vect.cpv(xm, ym), vect.cpv(xm, y0), segment, segment_data),
        0xE => segs(vect.cpv(xm, y0), vect.cpv(xm, ym), vect.cpv(x0, ym), segment, segment_data),
        else => {},
    }
}

pub fn cpMarchSoft(
    allocator: std.mem.Allocator,
    bounds: bb.cpBB,
    x_samples: usize,
    y_samples: usize,
    t: types.cpFloat,
    segment: cpMarchSegmentFunc,
    segment_data: ?*anyopaque,
    sample: cpMarchSampleFunc,
    sample_data: ?*anyopaque,
) void {
    marchCells(allocator, bounds, x_samples, y_samples, t, segment, segment_data, sample, sample_data, cellSoft);
}

pub fn cpMarchHard(
    allocator: std.mem.Allocator,
    bounds: bb.cpBB,
    x_samples: usize,
    y_samples: usize,
    t: types.cpFloat,
    segment: cpMarchSegmentFunc,
    segment_data: ?*anyopaque,
    sample: cpMarchSampleFunc,
    sample_data: ?*anyopaque,
) void {
    marchCells(allocator, bounds, x_samples, y_samples, t, segment, segment_data, sample, sample_data, cellHard);
}

test "march soft generates contour segments" {
    var segments = std.ArrayList(struct { a: vect.cpVect, b: vect.cpVect }).init(std.testing.allocator);
    defer segments.deinit();

    const sample = struct {
        fn call(p: vect.cpVect, _: ?*anyopaque) types.cpFloat {
            return 1.0 - vect.cpvlength(p);
        }
    }.call;

    const collect = struct {
        fn call(a: vect.cpVect, b: vect.cpVect, data: ?*anyopaque) void {
            const list = @as(*std.ArrayList(struct { a: vect.cpVect, b: vect.cpVect }), @ptrCast(@alignCast(data.?)));
            list.append(.{ .a = a, .b = b }) catch {};
        }
    }.call;

    const bounds = bb.cpBBNew(-1.0, -1.0, 1.0, 1.0);
    cpMarchSoft(std.testing.allocator, bounds, 4, 4, 0.0, collect, segments, sample, null);

    try std.testing.expect(segments.items.len > 0);
}
