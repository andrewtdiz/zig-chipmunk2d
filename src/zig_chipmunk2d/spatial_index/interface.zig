const std = @import("std");
const bb = @import("../core/bb.zig");

pub const BoundsFunc = fn (*const anyopaque, ?*const anyopaque) bb.cpBB;
pub const QueryFunc = fn (*const anyopaque, bb.cpBB, ?*anyopaque) void;
pub const EachFunc = fn (*const anyopaque, ?*anyopaque) void;

pub const Entry = struct {
    object: *const anyopaque,
    bounds: bb.cpBB,
};

pub const cpSpatialIndex = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const BoundsFunc,
    context: ?*const anyopaque,
    entries: std.ArrayList(Entry),

    pub fn init(allocator: std.mem.Allocator, bounds_func: *const BoundsFunc, context: ?*const anyopaque) cpSpatialIndex {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .entries = std.ArrayList(Entry).init(allocator),
        };
    }

    pub fn deinit(self: *cpSpatialIndex) void {
        self.entries.deinit();
    }

    pub fn insert(self: *cpSpatialIndex, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        try self.entries.append(.{ .object = object, .bounds = bounds });
    }

    pub fn remove(self: *cpSpatialIndex, object: *const anyopaque) void {
        var i: usize = 0;
        while (i < self.entries.items.len) : (i += 1) {
            if (self.entries.items[i].object == object) {
                _ = self.entries.swapRemove(i);
                return;
            }
        }
    }

    pub fn refreshBounds(self: *cpSpatialIndex) void {
        for (self.entries.items) |*entry| {
            entry.bounds = self.bounds_func(entry.object, self.context);
        }
    }

    pub fn reindex(self: *cpSpatialIndex) void {
        self.refreshBounds();
    }

    pub fn count(self: cpSpatialIndex) usize {
        return self.entries.items.len;
    }

    pub fn each(self: cpSpatialIndex, func: *const EachFunc, data: ?*anyopaque) void {
        for (self.entries.items) |entry| {
            func(entry.object, data);
        }
    }

    pub fn query(self: cpSpatialIndex, bounds: bb.cpBB, func: *const QueryFunc, data: ?*anyopaque) void {
        for (self.entries.items) |entry| {
            if (bb.cpBBIntersects(bounds, entry.bounds)) {
                func(entry.object, entry.bounds, data);
            }
        }
    }
};

pub fn bbForPointer(ptr: *const anyopaque, ctx: ?*const anyopaque) bb.cpBB {
    _ = ctx;
    const shape = @as(*const bb.cpBB, @ptrCast(ptr));
    return shape.*;
}

pub fn accumulateQuery(object: *const anyopaque, object_bounds: bb.cpBB, ctx: ?*anyopaque) void {
    _ = object;
    const writer = @as(*std.ArrayList(bb.cpBB), @ptrCast(ctx.?));
    writer.append(object_bounds) catch {};
}

pub fn accumulateEach(object: *const anyopaque, ctx: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(ctx.?));
    counter.* += @as(usize, 1);
    _ = object;
}

pub fn testSpatialIndexLifecycle() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var bounds = [3]bb.cpBB{
        bb.cpBBNew(0.0, 0.0, 1.0, 1.0),
        bb.cpBBNew(2.0, 2.0, 3.0, 3.0),
        bb.cpBBNew(-1.0, -1.0, 0.5, 0.5),
    };

    var index = cpSpatialIndex.init(allocator, bbForPointer, null);
    defer index.deinit();

    try index.insert(&bounds[0]);
    try index.insert(&bounds[1]);
    try index.insert(&bounds[2]);

    try std.testing.expectEqual(@as(usize, 3), index.count());

    var visited: usize = 0;
    index.each(accumulateEach, &visited);
    try std.testing.expectEqual(@as(usize, 3), visited);

    var matches = std.ArrayList(bb.cpBB).init(allocator);
    defer matches.deinit();
    index.query(bb.cpBBNew(-0.5, -0.5, 2.5, 2.5), accumulateQuery, &matches);
    try std.testing.expect(matches.items.len >= 2);

    index.remove(&bounds[1]);
    try std.testing.expectEqual(@as(usize, 2), index.count());

    bounds[0] = bb.cpBBNew(10.0, 10.0, 11.0, 11.0);
    index.refreshBounds();

    matches.clearRetainingCapacity();
    index.query(bb.cpBBNew(-2.0, -2.0, 2.0, 2.0), accumulateQuery, &matches);
    try std.testing.expectEqual(@as(usize, 1), matches.items.len);
}

pub fn testSpatialIndexReindex() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var bounds = [1]bb.cpBB{bb.cpBBNew(0.0, 0.0, 1.0, 1.0)};
    var index = cpSpatialIndex.init(allocator, bbForPointer, null);
    defer index.deinit();

    try index.insert(&bounds[0]);

    bounds[0] = bb.cpBBNew(5.0, 5.0, 6.0, 6.0);
    index.reindex();

    var matches = std.ArrayList(bb.cpBB).init(allocator);
    defer matches.deinit();
    index.query(bb.cpBBNew(4.0, 4.0, 7.0, 7.0), accumulateQuery, &matches);
    try std.testing.expectEqual(@as(usize, 1), matches.items.len);
}

comptime {
    std.testing.refAllDecls(@This());
}
