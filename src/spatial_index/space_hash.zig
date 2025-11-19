const std = @import("std");
const bb = @import("../core/bb.zig");
const types = @import("../core/types.zig");
const interface = @import("interface.zig");

const CellRange = struct {
    min_x: i64,
    max_x: i64,
    min_y: i64,
    max_y: i64,
};

fn cellIndex(value: types.cpFloat, cell_size: types.cpFloat) i64 {
    return @as(i64, @intFromFloat(std.math.floor(value / cell_size)));
}

fn encodeKey(x: i64, y: i64) u128 {
    const ux: u64 = @bitCast(@as(i64, x));
    const uy: u64 = @bitCast(@as(i64, y));
    return (@as(u128, ux) << 64) | @as(u128, uy);
}

fn removeFromBucket(bucket: *std.ArrayList(*const anyopaque), object: *const anyopaque) void {
    var i: usize = 0;
    while (i < bucket.items.len) : (i += 1) {
        if (bucket.items[i] == object) {
            _ = bucket.swapRemove(i);
            return;
        }
    }
}

fn cleanupBuckets(map: *std.AutoHashMap(u128, std.ArrayList(*const anyopaque))) void {
    var it = map.valueIterator();
    while (it.next()) |list| {
        list.deinit();
    }
    map.clearRetainingCapacity();
}

pub const cpSpaceHash = struct {
    allocator: std.mem.Allocator,
    cell_size: types.cpFloat,
    bounds_func: *const interface.BoundsFunc,
    context: ?*const anyopaque,
    buckets: std.AutoHashMap(u128, std.ArrayList(*const anyopaque)),
    bounds_map: std.AutoHashMap(*const anyopaque, bb.cpBB),

    pub fn init(
        allocator: std.mem.Allocator,
        cell_size: types.cpFloat,
        bounds_func: *const interface.BoundsFunc,
        context: ?*const anyopaque,
    ) cpSpaceHash {
        std.debug.assert(cell_size > 0.0);
        return .{
            .allocator = allocator,
            .cell_size = cell_size,
            .bounds_func = bounds_func,
            .context = context,
            .buckets = std.AutoHashMap(u128, std.ArrayList(*const anyopaque)).init(allocator),
            .bounds_map = std.AutoHashMap(*const anyopaque, bb.cpBB).init(allocator),
        };
    }

    pub fn deinit(self: *cpSpaceHash) void {
        cleanupBuckets(&self.buckets);
        self.buckets.deinit();
        self.bounds_map.deinit();
    }

    pub fn insert(self: *cpSpaceHash, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        try self.bounds_map.put(object, bounds);
        try self.insertIntoBuckets(object, bounds);
    }

    pub fn remove(self: *cpSpaceHash, object: *const anyopaque) void {
        if (self.bounds_map.fetchRemove(object)) |entry| {
            const bounds = entry.value;
            const cells = self.cellsFor(bounds);
            var y = cells.min_y;
            while (y <= cells.max_y) : (y += 1) {
                var x = cells.min_x;
                while (x <= cells.max_x) : (x += 1) {
                    const key = encodeKey(x, y);
                    if (self.buckets.getPtr(key)) |bucket| {
                        removeFromBucket(bucket, object);
                        if (bucket.items.len == 0) {
                            bucket.deinit();
                            _ = self.buckets.remove(key);
                        }
                    }
                }
            }
        }
    }

    pub fn refreshBounds(self: *cpSpaceHash) !void {
        var it = self.bounds_map.iterator();
        while (it.next()) |entry| {
            const object = entry.key_ptr.*;
            entry.value_ptr.* = self.bounds_func(object, self.context);
        }
        try self.rebuildBuckets();
    }

    pub fn reindex(self: *cpSpaceHash) !void {
        try self.refreshBounds();
    }

    pub fn each(self: *const cpSpaceHash, func: *const interface.EachFunc, data: ?*anyopaque) void {
        var it = self.bounds_map.iterator();
        while (it.next()) |entry| {
            func(entry.key_ptr.*, data);
        }
    }

    pub fn query(self: *const cpSpaceHash, bounds: bb.cpBB, func: *const interface.QueryFunc, data: ?*anyopaque) void {
        const cells = self.cellsFor(bounds);
        var visited = std.AutoHashMap(*const anyopaque, void).init(self.allocator);
        defer visited.deinit();

        var y = cells.min_y;
        while (y <= cells.max_y) : (y += 1) {
            var x = cells.min_x;
            while (x <= cells.max_x) : (x += 1) {
                const key = encodeKey(x, y);
                if (self.buckets.get(key)) |bucket| {
                    for (bucket.items) |object| {
                        const gop = visited.getOrPut(object) catch continue;
                        if (gop.found_existing) continue;
                        if (self.bounds_map.get(object)) |entry_bounds| {
                            if (bb.cpBBIntersects(bounds, entry_bounds)) {
                                func(object, entry_bounds, data);
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn count(self: *const cpSpaceHash) usize {
        return self.bounds_map.count();
    }

    fn insertIntoBuckets(self: *cpSpaceHash, object: *const anyopaque, bounds: bb.cpBB) !void {
        const cells = self.cellsFor(bounds);
        var y = cells.min_y;
        while (y <= cells.max_y) : (y += 1) {
            var x = cells.min_x;
            while (x <= cells.max_x) : (x += 1) {
                const key = encodeKey(x, y);
                var entry = try self.buckets.getOrPut(key);
                if (!entry.found_existing) {
                    entry.value_ptr.* = std.ArrayList(*const anyopaque).init(self.allocator);
                }
                try entry.value_ptr.append(object);
            }
        }
    }

    fn cellsFor(self: *const cpSpaceHash, bounds: bb.cpBB) CellRange {
        const min_x = cellIndex(bounds.l, self.cell_size);
        const max_x = cellIndex(bounds.r, self.cell_size);
        const min_y = cellIndex(bounds.b, self.cell_size);
        const max_y = cellIndex(bounds.t, self.cell_size);
        return .{ .min_x = min_x, .max_x = max_x, .min_y = min_y, .max_y = max_y };
    }

    fn rebuildBuckets(self: *cpSpaceHash) !void {
        cleanupBuckets(&self.buckets);
        var it = self.bounds_map.iterator();
        while (it.next()) |entry| {
            try self.insertIntoBuckets(entry.key_ptr.*, entry.value_ptr.*);
        }
    }
};

fn makeBox(x0: f64, y0: f64, x1: f64, y1: f64) bb.cpBB {
    return bb.cpBBNew(x0, y0, x1, y1);
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

test "cpSpaceHash basic queries" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var hash = cpSpaceHash.init(allocator, 1.0, boundsFromPointer, null);
    defer hash.deinit();

    var boxes = [_]bb.cpBB{
        makeBox(0.0, 0.0, 0.5, 0.5),
        makeBox(2.0, 2.0, 2.5, 2.5),
        makeBox(-1.0, -1.0, -0.5, -0.5),
    };

    try hash.insert(&boxes[0]);
    try hash.insert(&boxes[1]);
    try hash.insert(&boxes[2]);
    try std.testing.expectEqual(@as(usize, 3), hash.count());

    var hits: usize = 0;
    hash.query(makeBox(-0.75, -0.75, 0.75, 0.75), accumulate, &hits);
    try std.testing.expectEqual(@as(usize, 2), hits);

    hash.remove(&boxes[1]);
    try std.testing.expectEqual(@as(usize, 2), hash.count());
}

test "cpSpaceHash refresh updates buckets" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var hash = cpSpaceHash.init(allocator, 1.0, boundsFromPointer, null);
    defer hash.deinit();

    var box = makeBox(0.0, 0.0, 1.0, 1.0);
    try hash.insert(&box);

    box = makeBox(5.0, 5.0, 6.0, 6.0);
    try hash.refreshBounds();

    var hits: usize = 0;
    hash.query(makeBox(4.5, 4.5, 6.5, 6.5), accumulate, &hits);
    try std.testing.expectEqual(@as(usize, 1), hits);
}
