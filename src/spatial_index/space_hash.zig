const std = @import("std");
const bb = @import("../core/bb.zig");
const types = @import("../core/types.zig");
const spatial_interface = @import("interface.zig");

const CellKey = u128;

const Entry = struct {
    bounds: bb.cpBB,
    cells: std.ArrayList(CellKey),
    last_query: u64 = 0,
};

const CellBucket = struct {
    items: std.ArrayListUnmanaged(*const anyopaque) = .{},

    fn deinit(self: *CellBucket, allocator: std.mem.Allocator) void {
        self.items.deinit(allocator);
        self.items = .{};
    }

    fn add(self: *CellBucket, allocator: std.mem.Allocator, object: *const anyopaque) !void {
        try self.items.append(allocator, object);
    }

    fn remove(self: *CellBucket, allocator: std.mem.Allocator, object: *const anyopaque) void {
        var i: usize = 0;
        while (i < self.items.items.len) : (i += 1) {
            if (self.items.items[i] == object) {
                _ = self.items.swapRemove(i);
                break;
            }
        }

        if (self.items.items.len == 0 and self.items.capacity != 0) {
            self.deinit(allocator);
        }
    }
};

pub const Config = struct {
    cell_dim: types.cpFloat = 100.0,
    max_cells: usize = 256,
};

pub const cpSpaceHash = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const spatial_interface.BoundsFunc,
    context: ?*const anyopaque,
    entries: std.AutoHashMap(*const anyopaque, Entry),
    cells: std.AutoArrayHashMap(CellKey, CellBucket),
    config: Config,
    query_stamp: u64 = 1,

    pub fn init(
        allocator: std.mem.Allocator,
        bounds_func: *const spatial_interface.BoundsFunc,
        context: ?*const anyopaque,
        config: Config,
    ) cpSpaceHash {
        var cells = std.AutoArrayHashMap(CellKey, CellBucket).init(allocator);
        cells.ensureTotalCapacity(config.max_cells) catch {};
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .entries = std.AutoHashMap(*const anyopaque, Entry).init(allocator),
            .cells = cells,
            .config = config,
        };
    }

    pub fn deinit(self: *cpSpaceHash) void {
        var cell_it = self.cells.iterator();
        while (cell_it.next()) |cell| {
            cell.value_ptr.deinit(self.allocator);
        }
        self.cells.deinit();

        var entry_it = self.entries.iterator();
        while (entry_it.next()) |entry| {
            entry.value_ptr.cells.deinit(self.allocator);
        }
        self.entries.deinit();
    }

    pub fn insert(self: *cpSpaceHash, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        var entry = Entry{
            .bounds = bounds,
            .cells = .empty,
        };
        errdefer entry.cells.deinit(self.allocator);
        try self.populateEntry(&entry, object);
        try self.entries.put(object, entry);
    }

    pub fn remove(self: *cpSpaceHash, object: *const anyopaque) void {
        if (self.entries.fetchRemove(object)) |kv| {
            self.removeFromCells(kv.value, object);
            kv.value.cells.deinit(self.allocator);
        }
    }

    pub fn reindex(self: *cpSpaceHash) !void {
        var it = self.entries.iterator();
        while (it.next()) |entry| {
            const object = entry.key_ptr.*;
            self.removeFromCells(entry.value_ptr.*, object);
            entry.value_ptr.bounds = self.bounds_func(object, self.context);
            entry.value_ptr.cells.clearRetainingCapacity();
            try self.populateEntry(entry.value_ptr, object);
        }
    }

    pub fn query(self: *cpSpaceHash, bounds: bb.cpBB, func: *const spatial_interface.QueryFunc, data: ?*anyopaque) void {
        const range = self.cellRange(bounds);
        self.query_stamp +%= 1;
        var y = range.min_y;
        while (y <= range.max_y) : (y += 1) {
            var x = range.min_x;
            while (x <= range.max_x) : (x += 1) {
                const key = encodeCell(x, y);
                if (self.cells.getPtr(key)) |bucket| {
                    for (bucket.items.items) |object| {
                        if (self.entries.getPtr(object)) |entry| {
                            if (entry.last_query == self.query_stamp) continue;
                            entry.last_query = self.query_stamp;
                            if (bb.cpBBIntersects(entry.bounds, bounds)) {
                                func(object, entry.bounds, data);
                            }
                        }
                    }
                }
            }
        }
    }

    pub fn each(self: *cpSpaceHash, func: *const spatial_interface.EachFunc, data: ?*anyopaque) void {
        var it = self.entries.iterator();
        while (it.next()) |entry| {
            func(entry.key_ptr.*, data);
        }
    }

    pub fn count(self: cpSpaceHash) usize {
        return self.entries.count();
    }

    fn populateEntry(self: *cpSpaceHash, entry: *Entry, object: *const anyopaque) !void {
        const range = self.cellRange(entry.bounds);
        var y = range.min_y;
        while (y <= range.max_y) : (y += 1) {
            var x = range.min_x;
            while (x <= range.max_x) : (x += 1) {
                const key = encodeCell(x, y);
                try entry.cells.append(self.allocator, key);
                var gop = try self.cells.getOrPut(key);
                if (!gop.found_existing) {
                    gop.value_ptr.* = .{};
                }
                try gop.value_ptr.add(self.allocator, object);
            }
        }
    }

    fn removeFromCells(self: *cpSpaceHash, entry: Entry, object: *const anyopaque) void {
        for (entry.cells.items) |key| {
            if (self.cells.getPtr(key)) |bucket| {
                bucket.remove(self.allocator, object);
            }
        }
    }

    fn cellRange(self: *const cpSpaceHash, bounds: bb.cpBB) struct {
        min_x: i64,
        max_x: i64,
        min_y: i64,
        max_y: i64,
    } {
        const dim = if (self.config.cell_dim == 0.0) 1.0 else self.config.cell_dim;
        const min_x = @as(i64, @intFromFloat(@floor(bounds.l / dim)));
        const max_x = @as(i64, @intFromFloat(@floor(bounds.r / dim)));
        const min_y = @as(i64, @intFromFloat(@floor(bounds.b / dim)));
        const max_y = @as(i64, @intFromFloat(@floor(bounds.t / dim)));
        return .{ .min_x = min_x, .max_x = max_x, .min_y = min_y, .max_y = max_y };
    }
};

fn encodeCell(x: i64, y: i64) CellKey {
    const ux = @as(u64, @bitCast(x));
    const uy = @as(u64, @bitCast(y));
    return (@as(CellKey, ux) << 64) | @as(CellKey, uy);
}

fn makeBox(x0: f64, y0: f64, x1: f64, y1: f64) bb.cpBB {
    return bb.cpBBNew(x0, y0, x1, y1);
}

fn accumulateCount(_: *const anyopaque, _: bb.cpBB, ctx: ?*anyopaque) void {
    const counter = @as(*usize, @ptrCast(ctx.?));
    counter.* += 1;
}

test "cpSpaceHash basic query" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var bounds = [2]bb.cpBB{
        bb.cpBBNew(0.0, 0.0, 1.0, 1.0),
        bb.cpBBNew(5.0, 5.0, 6.0, 6.0),
    };
    var hash = cpSpaceHash.init(allocator, spatial_interface.bbForPointer, null, .{ .cell_dim = 2.0, .max_cells = 8 });
    defer hash.deinit();

    try hash.insert(&bounds[0]);
    try hash.insert(&bounds[1]);
    try std.testing.expectEqual(@as(usize, 2), hash.count());

    var matches: std.ArrayList(bb.cpBB) = .empty;
    defer matches.deinit(allocator);
    var accumulator_ctx = spatial_interface.AccumulateContext{ .list = &matches, .allocator = allocator };
    hash.query(bb.cpBBNew(-1.0, -1.0, 2.0, 2.0), spatial_interface.accumulateQuery, &accumulator_ctx);
    try std.testing.expectEqual(@as(usize, 1), matches.items.len);
}

test "cpSpaceHash reindex refreshes buckets" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var hash = cpSpaceHash.init(allocator, spatial_interface.bbForPointer, null, .{ .cell_dim = 1.0, .max_cells = 32 });
    defer hash.deinit();

    var box = makeBox(0.0, 0.0, 1.0, 1.0);
    try hash.insert(&box);

    box = makeBox(5.0, 5.0, 6.0, 6.0);
    try hash.reindex();

    var hits: usize = 0;
    hash.query(makeBox(4.5, 4.5, 6.5, 6.5), accumulateCount, &hits);
    try std.testing.expectEqual(@as(usize, 1), hits);
}
