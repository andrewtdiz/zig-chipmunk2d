const std = @import("std");
const bb = @import("../core/bb.zig");
const interface = @import("interface.zig");

fn lessThan(_: void, lhs: interface.Entry, rhs: interface.Entry) bool {
    return lhs.bounds.l < rhs.bounds.l;
}

pub const cpSweep1D = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const interface.BoundsFunc,
    context: ?*const anyopaque,
    entries: std.ArrayList(interface.Entry),

    pub fn init(
        allocator: std.mem.Allocator,
        bounds_func: *const interface.BoundsFunc,
        context: ?*const anyopaque,
    ) cpSweep1D {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .entries = std.ArrayList(interface.Entry).init(allocator),
        };
    }

    pub fn deinit(self: *cpSweep1D) void {
        self.entries.deinit();
    }

    pub fn insert(self: *cpSweep1D, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        try self.entries.append(.{ .object = object, .bounds = bounds });
        sortEntries(self);
    }

    pub fn remove(self: *cpSweep1D, object: *const anyopaque) void {
        var i: usize = 0;
        while (i < self.entries.items.len) : (i += 1) {
            if (self.entries.items[i].object == object) {
                _ = self.entries.swapRemove(i);
                sortEntries(self);
                break;
            }
        }
    }

    pub fn refreshBounds(self: *cpSweep1D) void {
        for (self.entries.items) |*entry| {
            entry.bounds = self.bounds_func(entry.object, self.context);
        }
        sortEntries(self);
    }

    pub fn reindex(self: *cpSweep1D) void {
        self.refreshBounds();
    }

    pub fn each(self: *const cpSweep1D, func: *const interface.EachFunc, data: ?*anyopaque) void {
        for (self.entries.items) |entry| {
            func(entry.object, data);
        }
    }

    pub fn query(self: *const cpSweep1D, bounds: bb.cpBB, func: *const interface.QueryFunc, data: ?*anyopaque) void {
        for (self.entries.items) |entry| {
            if (entry.bounds.l > bounds.r) break;
            if (bb.cpBBIntersects(bounds, entry.bounds)) {
                func(entry.object, entry.bounds, data);
            }
        }
    }

    pub fn count(self: *const cpSweep1D) usize {
        return self.entries.items.len;
    }
};

fn sortEntries(self: *cpSweep1D) void {
    std.sort.block(interface.Entry, self.entries.items, {}, lessThan);
}

fn boundsFromPointer(ptr: *const anyopaque, _: ?*const anyopaque) bb.cpBB {
    const casted = @as(*const bb.cpBB, @ptrCast(ptr));
    return casted.*;
}

fn accumulate(object: *const anyopaque, object_bounds: bb.cpBB, data: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(data.?));
    counter.* += 1;
    _ = object_bounds;
    _ = object;
}

fn recordEach(object: *const anyopaque, ctx: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(ctx.?));
    counter.* += 1;
    _ = object;
}

fn makeBox(x0: f64, y0: f64, x1: f64, y1: f64) bb.cpBB {
    return bb.cpBBNew(x0, y0, x1, y1);
}

test "cpSweep1D insertion and query" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var sweep = cpSweep1D.init(allocator, boundsFromPointer, null);
    defer sweep.deinit();

    var boxes = [_]bb.cpBB{
        makeBox(0.0, 0.0, 1.0, 1.0),
        makeBox(3.0, 3.0, 4.0, 4.0),
        makeBox(1.5, 1.5, 2.0, 2.0),
    };

    try sweep.insert(&boxes[0]);
    try sweep.insert(&boxes[1]);
    try sweep.insert(&boxes[2]);
    try std.testing.expectEqual(@as(usize, 3), sweep.count());

    var hits: usize = 0;
    sweep.query(makeBox(0.5, 0.5, 2.5, 2.5), accumulate, &hits);
    try std.testing.expectEqual(@as(usize, 2), hits);
}

test "cpSweep1D refresh sorts entries" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var sweep = cpSweep1D.init(allocator, boundsFromPointer, null);
    defer sweep.deinit();

    var box = makeBox(0.0, 0.0, 1.0, 1.0);
    try sweep.insert(&box);
    box = makeBox(5.0, 5.0, 6.0, 6.0);
    sweep.refreshBounds();

    var seen: usize = 0;
    sweep.each(recordEach, &seen);
    try std.testing.expectEqual(@as(usize, 1), seen);

    var hits: usize = 0;
    sweep.query(makeBox(4.5, 4.5, 6.5, 6.5), accumulate, &hits);
    try std.testing.expectEqual(@as(usize, 1), hits);
}
