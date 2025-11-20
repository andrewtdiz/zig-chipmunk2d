const std = @import("std");
const bb = @import("../core/bb.zig");
const spatial_interface = @import("interface.zig");

const Entry = struct {
    object: *const anyopaque,
    bounds: bb.cpBB,
};

pub const cpSweep1D = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const spatial_interface.BoundsFunc,
    context: ?*const anyopaque,
    entries: std.ArrayList(Entry),
    dirty: bool = false,

    pub fn init(
        allocator: std.mem.Allocator,
        bounds_func: *const spatial_interface.BoundsFunc,
        context: ?*const anyopaque,
    ) cpSweep1D {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .entries = .empty,
        };
    }

    pub fn deinit(self: *cpSweep1D) void {
        self.entries.deinit(self.allocator);
    }

    pub fn insert(self: *cpSweep1D, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        try self.entries.append(self.allocator, .{ .object = object, .bounds = bounds });
        self.dirty = true;
    }

    pub fn remove(self: *cpSweep1D, object: *const anyopaque) void {
        var i: usize = 0;
        while (i < self.entries.items.len) : (i += 1) {
            if (self.entries.items[i].object == object) {
                _ = self.entries.swapRemove(i);
                self.dirty = true;
                break;
            }
        }
    }

    pub fn reindex(self: *cpSweep1D) !void {
        for (self.entries.items) |*entry| {
            entry.bounds = self.bounds_func(entry.object, self.context);
        }
        self.dirty = true;
        try self.ensureSorted();
    }

    pub fn query(self: *cpSweep1D, bounds: bb.cpBB, func: *const spatial_interface.QueryFunc, data: ?*anyopaque) void {
        self.ensureSorted() catch return;
        for (self.entries.items) |entry| {
            if (entry.bounds.l > bounds.r) break;
            if (!bb.cpBBIntersects(entry.bounds, bounds)) continue;
            func(entry.object, entry.bounds, data);
        }
    }

    pub fn each(self: *const cpSweep1D, func: *const spatial_interface.EachFunc, data: ?*anyopaque) void {
        for (self.entries.items) |entry| {
            func(entry.object, data);
        }
    }

    pub fn count(self: cpSweep1D) usize {
        return self.entries.items.len;
    }

    fn ensureSorted(self: *cpSweep1D) !void {
        if (!self.dirty) return;
        std.sort.sort(Entry, self.entries.items, {}, struct {
            fn lessThan(_: void, a: Entry, b: Entry) bool {
                return a.bounds.l < b.bounds.l;
            }
        }.lessThan);
        self.dirty = false;
    }
};

fn boundsFromPointer(ptr: *const anyopaque, _: ?*const anyopaque) bb.cpBB {
    const casted = @as(*const bb.cpBB, @ptrCast(ptr));
    return casted.*;
}

fn accumulateHits(_: *const anyopaque, _: bb.cpBB, ctx: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(ctx.?));
    counter.* += 1;
}

fn recordEach(_: *const anyopaque, ctx: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(ctx.?));
    counter.* += 1;
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
    sweep.query(makeBox(0.5, 0.5, 2.5, 2.5), accumulateHits, &hits);
    try std.testing.expectEqual(@as(usize, 2), hits);
}

test "cpSweep1D reindex resorts entries" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var sweep = cpSweep1D.init(allocator, boundsFromPointer, null);
    defer sweep.deinit();

    var box = makeBox(0.0, 0.0, 1.0, 1.0);
    try sweep.insert(&box);

    box = makeBox(5.0, 5.0, 6.0, 6.0);
    try sweep.reindex();

    var visited: usize = 0;
    sweep.each(recordEach, &visited);
    try std.testing.expectEqual(@as(usize, 1), visited);

    var hits: usize = 0;
    sweep.query(makeBox(4.5, 4.5, 6.5, 6.5), accumulateHits, &hits);
    try std.testing.expectEqual(@as(usize, 1), hits);
}
