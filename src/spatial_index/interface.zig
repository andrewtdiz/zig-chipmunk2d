const std = @import("std");
const bb = @import("../core/bb.zig");
const types = @import("../core/types.zig");

pub const BoundsFunc = fn (*const anyopaque, ?*const anyopaque) bb.cpBB;
pub const QueryFunc = fn (*const anyopaque, bb.cpBB, ?*anyopaque) void;
pub const EachFunc = fn (*const anyopaque, ?*anyopaque) void;

const null_index: usize = std.math.maxInt(usize);

const Node = struct {
    parent: usize = null_index,
    left: usize = null_index,
    right: usize = null_index,
    height: isize = 0,
    object: ?*const anyopaque = null,
    bounds: bb.cpBB = bb.cpBBNew(0.0, 0.0, 0.0, 0.0),

    fn isLeaf(self: Node) bool {
        return self.left == null_index;
    }
};

fn fatten(bounds: bb.cpBB, margin: types.cpFloat) bb.cpBB {
    return bb.cpBBNew(bounds.l - margin, bounds.b - margin, bounds.r + margin, bounds.t + margin);
}

pub const cpSpatialIndex = struct {
    allocator: std.mem.Allocator,
    bounds_func: *const BoundsFunc,
    context: ?*const anyopaque,
    nodes: std.ArrayList(Node),
    free_list: usize = null_index,
    root: usize = null_index,
    lookup: std.AutoHashMap(*const anyopaque, usize),
    stack: std.ArrayList(usize),
    fat_margin: types.cpFloat = 2.0,

    pub fn init(allocator: std.mem.Allocator, bounds_func: *const BoundsFunc, context: ?*const anyopaque) cpSpatialIndex {
        return .{
            .allocator = allocator,
            .bounds_func = bounds_func,
            .context = context,
            .nodes = std.ArrayList(Node).init(allocator),
            .lookup = std.AutoHashMap(*const anyopaque, usize).init(allocator),
            .stack = std.ArrayList(usize).init(allocator),
        };
    }

    pub fn deinit(self: *cpSpatialIndex) void {
        self.nodes.deinit();
        self.lookup.deinit();
        self.stack.deinit();
    }

    pub fn insert(self: *cpSpatialIndex, object: *const anyopaque) !void {
        const bounds = fatten(self.bounds_func(object, self.context), self.fat_margin);
        const leaf_index = try self.allocateLeaf(object, bounds);
        errdefer {
            _ = self.lookup.remove(object);
            self.freeNode(leaf_index);
        }
        try self.insertLeaf(leaf_index);
    }

    pub fn remove(self: *cpSpatialIndex, object: *const anyopaque) void {
        if (self.lookup.fetchRemove(object)) |entry| {
            const index = entry.value;
            self.removeLeaf(index);
            self.freeNode(index);
        }
    }

    pub fn reindex(self: *cpSpatialIndex) !void {
        var it = self.lookup.iterator();
        while (it.next()) |entry| {
            const object = entry.key_ptr.*;
            const node_index = entry.value_ptr.*;
            const node = &self.nodes.items[node_index];
            const new_bounds = fatten(self.bounds_func(object, self.context), self.fat_margin);
            if (!bb.cpBBContainsBB(node.bounds, new_bounds)) {
                self.removeLeaf(node_index);
                node.bounds = new_bounds;
                try self.insertLeaf(node_index);
            } else {
                node.bounds = new_bounds;
                self.syncAncestors(node.parent);
            }
        }
    }

    pub fn count(self: cpSpatialIndex) usize {
        return self.lookup.count();
    }

    pub fn each(self: cpSpatialIndex, func: *const EachFunc, data: ?*anyopaque) void {
        var it = self.lookup.iterator();
        while (it.next()) |entry| {
            func(entry.key_ptr.*, data);
        }
    }

    pub fn query(self: *cpSpatialIndex, bounds: bb.cpBB, func: *const QueryFunc, data: ?*anyopaque) void {
        if (self.root == null_index) return;
        self.stack.clearRetainingCapacity();
        self.stack.append(self.root) catch return;

        while (self.stack.items.len > 0) {
            const index = self.stack.pop();
            const node = self.nodes.items[index];
            if (!bb.cpBBIntersects(node.bounds, bounds)) continue;
            if (node.isLeaf()) {
                func(node.object.?, node.bounds, data);
            } else {
                if (node.left != null_index) self.stack.append(node.left) catch return;
                if (node.right != null_index) self.stack.append(node.right) catch return;
            }
        }
    }

    fn allocateNode(self: *cpSpatialIndex) !usize {
        if (self.free_list != null_index) {
            const index = self.free_list;
            self.free_list = self.nodes.items[index].parent;
            return index;
        }

        const index = self.nodes.items.len;
        try self.nodes.append(.{});
        return index;
    }

    fn freeNode(self: *cpSpatialIndex, index: usize) void {
        self.nodes.items[index] = .{ .parent = self.free_list };
        self.free_list = index;
    }

    fn allocateLeaf(self: *cpSpatialIndex, object: *const anyopaque, bounds: bb.cpBB) !usize {
        const index = try self.allocateNode();
        self.nodes.items[index] = .{ .object = object, .bounds = bounds };
        try self.lookup.put(object, index);
        return index;
    }

    fn insertLeaf(self: *cpSpatialIndex, leaf_index: usize) !void {
        if (self.root == null_index) {
            self.root = leaf_index;
            self.nodes.items[leaf_index].parent = null_index;
            return;
        }

        var index = self.root;
        const leaf_bounds = self.nodes.items[leaf_index].bounds;
        while (!self.nodes.items[index].isLeaf()) {
            const left = self.nodes.items[index].left;
            const right = self.nodes.items[index].right;

            const area = bb.cpBBArea(self.nodes.items[index].bounds);
            const combined = bb.cpBBMerge(self.nodes.items[index].bounds, leaf_bounds);
            const combined_area = bb.cpBBArea(combined);
            const cost = 2.0 * combined_area;
            const inheritance = 2.0 * (combined_area - area);

            const cost_left = self.branchCost(left, leaf_bounds, inheritance);
            const cost_right = self.branchCost(right, leaf_bounds, inheritance);

            if (cost < cost_left and cost < cost_right) break;

            index = if (cost_left < cost_right) left else right;
        }

        const sibling = index;
        const old_parent = self.nodes.items[sibling].parent;
        const new_parent_index = try self.allocateNode();
        self.nodes.items[new_parent_index] = .{
            .parent = old_parent,
            .left = sibling,
            .right = leaf_index,
            .height = self.nodes.items[sibling].height + 1,
            .bounds = bb.cpBBMerge(self.nodes.items[sibling].bounds, leaf_bounds),
        };

        self.nodes.items[sibling].parent = new_parent_index;
        self.nodes.items[leaf_index].parent = new_parent_index;

        if (old_parent == null_index) {
            self.root = new_parent_index;
        } else {
            var parent = &self.nodes.items[old_parent];
            if (parent.left == sibling) {
                parent.left = new_parent_index;
            } else {
                parent.right = new_parent_index;
            }
        }

        self.syncAncestors(new_parent_index);
    }

    fn branchCost(self: *cpSpatialIndex, node_index: usize, bounds: bb.cpBB, inheritance: types.cpFloat) types.cpFloat {
        const node = self.nodes.items[node_index];
        const merged = bb.cpBBMerge(node.bounds, bounds);
        if (node.isLeaf()) {
            return bb.cpBBArea(merged) + inheritance;
        }

        const old_area = bb.cpBBArea(node.bounds);
        return (bb.cpBBArea(merged) - old_area) + inheritance;
    }

    fn removeLeaf(self: *cpSpatialIndex, leaf_index: usize) void {
        if (leaf_index == self.root) {
            self.root = null_index;
            return;
        }

        const parent_index = self.nodes.items[leaf_index].parent;
        const parent = self.nodes.items[parent_index];
        const grand = parent.parent;
        const sibling = if (parent.left == leaf_index) parent.right else parent.left;

        if (grand == null_index) {
            self.root = sibling;
            self.nodes.items[sibling].parent = null_index;
        } else {
            var grand_node = &self.nodes.items[grand];
            if (grand_node.left == parent_index) {
                grand_node.left = sibling;
            } else {
                grand_node.right = sibling;
            }
            self.nodes.items[sibling].parent = grand;
        }

        self.freeNode(parent_index);
        self.syncAncestors(grand);
    }

    fn syncAncestors(self: *cpSpatialIndex, start: usize) void {
        var index = start;
        while (index != null_index) {
            const balanced = self.balance(index);
            self.pullBounds(balanced);
            index = self.nodes.items[balanced].parent;
        }
    }

    fn pullBounds(self: *cpSpatialIndex, index: usize) void {
        if (index == null_index) return;
        const left = self.nodes.items[index].left;
        const right = self.nodes.items[index].right;
        if (left == null_index or right == null_index) return;
        const left_node = self.nodes.items[left];
        const right_node = self.nodes.items[right];
        self.nodes.items[index].bounds = bb.cpBBMerge(left_node.bounds, right_node.bounds);
        self.nodes.items[index].height = 1 + @max(left_node.height, right_node.height);
    }

    fn balance(self: *cpSpatialIndex, iA: usize) usize {
        const A = self.nodes.items[iA];
        if (A.isLeaf() or A.height < 2) return iA;

        const iB = A.left;
        const iC = A.right;
        const B = self.nodes.items[iB];
        const C = self.nodes.items[iC];

        const balance_factor = @as(isize, C.height) - @as(isize, B.height);
        if (balance_factor > 1) {
            const iF = self.nodes.items[iC].left;
            const iG = self.nodes.items[iC].right;

            self.nodes.items[iC].left = iA;
            self.nodes.items[iC].parent = A.parent;
            self.nodes.items[iA].parent = iC;

            if (self.nodes.items[iC].parent == null_index) {
                self.root = iC;
            } else {
                var parent = &self.nodes.items[self.nodes.items[iC].parent];
                if (parent.left == iA) {
                    parent.left = iC;
                } else {
                    parent.right = iC;
                }
            }

            if (self.nodes.items[iF].height > self.nodes.items[iG].height) {
                self.nodes.items[iC].right = iF;
                self.nodes.items[iA].right = iG;
                self.nodes.items[iG].parent = iA;
                self.nodes.items[iF].parent = iC;
            } else {
                self.nodes.items[iC].right = iG;
                self.nodes.items[iA].right = iF;
                self.nodes.items[iF].parent = iA;
                self.nodes.items[iG].parent = iC;
            }

            self.pullBounds(iA);
            self.pullBounds(iC);
            return iC;
        }

        if (balance_factor < -1) {
            const iD = self.nodes.items[iB].left;
            const iE = self.nodes.items[iB].right;

            self.nodes.items[iB].left = iA;
            self.nodes.items[iB].parent = A.parent;
            self.nodes.items[iA].parent = iB;

            if (self.nodes.items[iB].parent == null_index) {
                self.root = iB;
            } else {
                var parent = &self.nodes.items[self.nodes.items[iB].parent];
                if (parent.left == iA) {
                    parent.left = iB;
                } else {
                    parent.right = iB;
                }
            }

            if (self.nodes.items[iD].height > self.nodes.items[iE].height) {
                self.nodes.items[iB].right = iD;
                self.nodes.items[iA].left = iE;
                self.nodes.items[iE].parent = iA;
                self.nodes.items[iD].parent = iB;
            } else {
                self.nodes.items[iB].right = iE;
                self.nodes.items[iA].left = iD;
                self.nodes.items[iD].parent = iA;
                self.nodes.items[iE].parent = iB;
            }

            self.pullBounds(iA);
            self.pullBounds(iB);
            return iB;
        }

        return iA;
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
    counter.* += 1;
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
    try index.reindex();

    matches.clearRetainingCapacity();
    index.query(bb.cpBBNew(9.0, 9.0, 12.0, 12.0), accumulateQuery, &matches);
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
    try index.reindex();

    var matches = std.ArrayList(bb.cpBB).init(allocator);
    defer matches.deinit();
    index.query(bb.cpBBNew(4.0, 4.0, 7.0, 7.0), accumulateQuery, &matches);
    try std.testing.expectEqual(@as(usize, 1), matches.items.len);
}

comptime {
    std.testing.refAllDecls(@This());
}
