const std = @import("std");

pub fn ObjectPool(comptime T: type) type {
    return struct {
        const Self = @This();

        allocator: std.mem.Allocator,
        storage: std.ArrayList(*T),
        free_list: std.ArrayList(*T),

        pub fn init(allocator: std.mem.Allocator, initial_capacity: usize) !Self {
            var pool = Self{
                .allocator = allocator,
                .storage = std.ArrayList(*T).init(allocator),
                .free_list = std.ArrayList(*T).init(allocator),
            };
            errdefer pool.deinit();

            try pool.storage.ensureTotalCapacity(initial_capacity);
            try pool.free_list.ensureTotalCapacity(initial_capacity);
            return pool;
        }

        pub fn deinit(self: *Self) void {
            for (self.storage.items) |ptr| {
                self.allocator.destroy(ptr);
            }
            self.storage.deinit();
            self.free_list.deinit();
        }

        pub fn acquire(self: *Self) !*T {
            if (self.free_list.popOrNull()) |ptr| {
                return ptr;
            }
            const ptr = try self.allocator.create(T);
            try self.storage.append(ptr);
            return ptr;
        }

        pub fn release(self: *Self, ptr: *T) void {
            self.free_list.append(ptr) catch {};
        }
    };
}
