const std = @import("std");
const bb = @import("../core/bb.zig");
const spatial_interface = @import("interface.zig");

pub const cpBBTree = struct {
    index: spatial_interface.cpSpatialIndex,

    pub fn init(allocator: std.mem.Allocator, bounds_func: *const spatial_interface.BoundsFunc, context: ?*const anyopaque) cpBBTree {
        return .{ .index = spatial_interface.cpSpatialIndex.init(allocator, bounds_func, context) };
    }

    pub fn deinit(self: *cpBBTree) void {
        self.index.deinit();
    }

    pub fn insert(self: *cpBBTree, object: *const anyopaque) !void {
        try self.index.insert(object);
    }

    pub fn remove(self: *cpBBTree, object: *const anyopaque) void {
        self.index.remove(object);
    }

    pub fn reindex(self: *cpBBTree) !void {
        try self.index.reindex();
    }

    pub fn query(self: cpBBTree, bounds: bb.cpBB, func: *const spatial_interface.QueryFunc, data: ?*anyopaque) void {
        self.index.query(bounds, func, data);
    }

    pub fn count(self: cpBBTree) usize {
        return self.index.count();
    }

    pub fn each(self: *cpBBTree, func: *const spatial_interface.EachFunc, data: ?*anyopaque) void {
        self.index.each(func, data);
    }
};

pub fn testBBTreeDelegatesToIndex() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    var bounds = [2]bb.cpBB{bb.cpBBNew(0.0, 0.0, 1.0, 1.0), bb.cpBBNew(2.0, 2.0, 3.0, 3.0)};

    var tree = cpBBTree.init(allocator, spatial_interface.BoundsFunc(spatial_interface.bbForPointer), null);
    defer tree.deinit();

    try tree.insert(&bounds[0]);
    try tree.insert(&bounds[1]);

    try std.testing.expectEqual(@as(usize, 2), tree.count());

    bounds[0] = bb.cpBBNew(4.0, 4.0, 5.0, 5.0);
    try tree.reindex();

    var matches = std.ArrayList(bb.cpBB).init(allocator);
    defer matches.deinit();
    tree.query(bb.cpBBNew(3.5, 3.5, 5.5, 5.5), spatial_interface.accumulateQuery, &matches);
    try std.testing.expectEqual(@as(usize, 1), matches.items.len);
}

comptime {
    std.testing.refAllDecls(@This());
}
