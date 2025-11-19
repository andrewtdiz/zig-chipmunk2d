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

    pub fn init(allocator: std.mem.Allocator, bounds_func: *const spatial_interface.BoundsFunc, context: ?*const anyopaque) cpSweep1D {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .entries = std.ArrayList(Entry).init(allocator),
        };
    }

    pub fn deinit(self: *cpSweep1D) void {
        self.entries.deinit();
    }

    pub fn insert(self: *cpSweep1D, object: *const anyopaque) !void {
        const bounds = self.bounds_func(object, self.context);
        try self.entries.append(.{ .object = object, .bounds = bounds });
        self.dirty = true;
    }

    pub fn remove(self: *cpSweep1D, object: *const anyopaque) void {
        var i: usize = 0;
        while (i < self.entries.items.len) : (i += 1) {
            if (self.entries.items[i].object == object) {
                _ = self.entries.swapRemove(i);
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

    pub fn each(self: *cpSweep1D, func: *const spatial_interface.EachFunc, data: ?*anyopaque) void {
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

test "sweep1d sorts entries" {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var storage = [2]bb.cpBB{ bb.cpBBNew(3, 3, 4, 4), bb.cpBBNew(0, 0, 1, 1) };
    var sweep = cpSweep1D.init(allocator, spatial_interface.bbForPointer, null);
    defer sweep.deinit();

    try sweep.insert(&storage[0]);
    try sweep.insert(&storage[1]);

    var visited: usize = 0;
    sweep.query(bb.cpBBNew(-1, -1, 2, 2), spatial_interface.accumulateQuery, &visited);
    try std.testing.expect(visited > 0);
}
