const std = @import("std");
const types = @import("../core/types.zig");
const bb = @import("../core/bb.zig");
const vect = @import("../core/vect.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("../collision/collision.zig");
const arbiter = @import("../collision/arbiter.zig");
const body_mod = @import("body.zig");

pub fn makeStepper(comptime Space: type, comptime Helpers: type) type {
    return struct {
        const Handler = @TypeOf(@as(*Space, undefined).handler);
        const CachedArbiter = @TypeOf(@as(*Space, undefined).arbiter_cache).Value;

        pub fn step(space: *Space, dt: types.cpFloat) void {
            if (dt == 0.0) return;

            space.prev_dt = space.curr_dt;
            space.curr_dt = dt;
            space.stamp +%= 1;

            const dt_coef: types.cpFloat = if (space.prev_dt != 0.0) dt / space.prev_dt else 0.0;

            recycleArbiters(space);
            integratePositions(space, dt);
            cacheAllShapes(space);
            space.dynamic_index.reindex() catch {};
            broadPhase(space);
            processComponents(space, dt);

            preStepArbiters(space, dt);
            runConstraintCallback(space, dt, dt_coef, .preStep);
            updateVelocities(space, dt);
            applyCachedArbiterImpulses(space, dt_coef);
            runConstraintCallback(space, dt, dt_coef, .applyCachedImpulse);

            var iteration: usize = 0;
            while (iteration < space.iterations) : (iteration += 1) {
                runConstraintCallback(space, dt, dt_coef, .applyImpulse);
                resolveArbiters(space);
            }

            runConstraintCallback(space, dt, dt_coef, .postStep);
            runPostSolve(space);
            runSeparations(space);
            runPostSteps(space);
            syncArbiterCache(space);
            pruneArbiterCache(space);
        }

        const Callback = enum { preStep, applyCachedImpulse, applyImpulse, postStep };

        fn runConstraintCallback(space: *Space, dt: types.cpFloat, dt_coef: types.cpFloat, comptime which: Callback) void {
            for (space.constraints.items) |entry| {
                switch (which) {
                    .preStep => if (entry.ops.preStep) |fn_ptr| fn_ptr(entry.payload, dt),
                    .applyCachedImpulse => if (entry.ops.applyCachedImpulse) |fn_ptr| fn_ptr(entry.payload, dt_coef),
                    .applyImpulse => if (entry.ops.applyImpulse) |fn_ptr| fn_ptr(entry.payload),
                    .postStep => if (entry.ops.postStep) |fn_ptr| fn_ptr(entry.payload),
                }
            }
        }

        fn updateVelocities(space: *Space, dt: types.cpFloat) void {
            const damping_factor = std.math.pow(types.cpFloat, space.damping, dt);
            for (space.bodies.items) |body| {
                if (body.sleeping) continue;
                body.updateVelocity(space.gravity, damping_factor, dt);
            }
        }

        fn integratePositions(space: *Space, dt: types.cpFloat) void {
            for (space.bodies.items) |body| {
                if (body.sleeping) continue;
                body.updatePosition(dt);
            }
        }

        fn cacheAllShapes(space: *Space) void {
            for (space.shapes.items) |shape| {
                Helpers.cacheShape(shape);
            }
        }

        fn broadPhase(space: *Space) void {
            for (space.dynamic_shapes.items) |shape| {
                var ctx = QueryContext{ .space = space, .primary = shape, .static_query = false };
                space.dynamic_index.query(shape.bbValue(), queryPairs, &ctx);
                ctx.static_query = true;
                space.static_index.query(shape.bbValue(), queryPairs, &ctx);
            }
        }

        pub fn processComponents(space: *Space, dt: types.cpFloat) void {
            _ = dt;

            var index_map = std.AutoHashMap(*body_mod.cpBody, usize).init(space.allocator);
            defer index_map.deinit();

            var parents = std.ArrayList(usize).init(space.allocator);
            defer parents.deinit();
            var ranks = std.ArrayList(u8).init(space.allocator);
            defer ranks.deinit();
            var anchored = std.ArrayList(bool).init(space.allocator);
            defer anchored.deinit();

            for (space.bodies.items) |body| {
                if (body.body_type != .dynamic) continue;
                parents.append(space.allocator, parents.items.len) catch {};
                ranks.append(space.allocator, 0) catch {};
                anchored.append(space.allocator, false) catch {};
                index_map.put(space.allocator, body, parents.items.len - 1) catch {};
            }

            const unionFind = struct {
                parents: []usize,
                ranks: []u8,
                fn find(self: @This(), idx: usize) usize {
                    var i = idx;
                    while (self.parents[i] != i) {
                        self.parents[i] = self.parents[self.parents[i]];
                        i = self.parents[i];
                    }
                    return i;
                }
                fn unite(self: @This(), a: usize, b: usize) void {
                    const ra = self.find(a);
                    const rb = self.find(b);
                    if (ra == rb) return;
                    if (self.ranks[ra] < self.ranks[rb]) {
                        self.parents[ra] = rb;
                    } else if (self.ranks[ra] > self.ranks[rb]) {
                        self.parents[rb] = ra;
                    } else {
                        self.parents[rb] = ra;
                        self.ranks[ra] += 1;
                    }
                }
            }{ .parents = parents.items, .ranks = ranks.items };

            // Union bodies connected via arbiters and constraints.
            for (space.arbiters.items) |arb_ref| {
                const a_body = arb_ref.shape_a.body;
                const b_body = arb_ref.shape_b.body;
                const a_dyn = a_body.body_type == .dynamic;
                const b_dyn = b_body.body_type == .dynamic;
                const a_idx = if (a_dyn) index_map.get(a_body) else null;
                const b_idx = if (b_dyn) index_map.get(b_body) else null;
                if (a_dyn and b_dyn and a_idx != null and b_idx != null) {
                    unionFind.unite(a_idx.?, b_idx.?);
                } else if (a_dyn and a_idx != null) {
                    anchored.items[a_idx.?] = true;
                } else if (b_dyn and b_idx != null) {
                    anchored.items[b_idx.?] = true;
                }
            }

            for (space.constraints.items) |entry| {
                const a_body = entry.constraint.a;
                const b_body = entry.constraint.b;
                const a_dyn = a_body.body_type == .dynamic;
                const b_dyn = b_body.body_type == .dynamic;
                const a_idx = if (a_dyn) index_map.get(a_body) else null;
                const b_idx = if (b_dyn) index_map.get(b_body) else null;
                if (a_dyn and b_dyn and a_idx != null and b_idx != null) {
                    unionFind.unite(a_idx.?, b_idx.?);
                } else if (a_dyn and a_idx != null) {
                    anchored.items[a_idx.?] = true;
                } else if (b_dyn and b_idx != null) {
                    anchored.items[b_idx.?] = true;
                }
            }

            var component_energy = std.ArrayList(types.cpFloat).init(space.allocator);
            defer component_energy.deinit();
            var component_anchored = std.ArrayList(bool).init(space.allocator);
            defer component_anchored.deinit();
            component_energy.resize(parents.items.len) catch {};
            component_anchored.resize(parents.items.len) catch {};
            @memset(component_energy.items, 0.0);
            @memset(component_anchored.items, false);

            for (space.bodies.items) |body| {
                if (body.body_type != .dynamic) continue;
                const idx = index_map.get(body) orelse continue;
                const root = unionFind.find(idx);
                component_energy.items[root] += body.kineticEnergy();
                component_anchored.items[root] = component_anchored.items[root] or anchored.items[idx];
            }

            for (space.bodies.items) |body| {
                if (body.body_type != .dynamic) continue;
                const idx = index_map.get(body) orelse continue;
                const root = unionFind.find(idx);
                const should_sleep = !component_anchored.items[root] and component_energy.items[root] < space.sleep_energy_threshold;
                body.sleeping = should_sleep;
                if (should_sleep) {
                    body.v = vect.cpvzero;
                    body.w = 0.0;
                    body.v_bias = vect.cpvzero;
                    body.w_bias = 0.0;
                }
            }
        }

        fn queryPairs(object: *const anyopaque, _: bb.cpBB, ctx_ptr: ?*anyopaque) void {
            const ctx = @as(*QueryContext, @ptrCast(ctx_ptr.?));
            const other = @as(*shape_base.cpShape, @ptrCast(object));
            if (other == ctx.primary) return;
            if (!ctx.static_query) {
                if (@intFromPtr(other) <= @intFromPtr(ctx.primary)) return;
            }
            if (ctx.primary.body.sleeping and other.body.sleeping) return;
            if (shape_base.cpShapeFilter.reject(ctx.primary.filter, other.filter)) return;
            const handler = selectHandler(ctx.space, ctx.primary, other);
            const result = collision.collide(&ctx.space.collision_cache, ctx.primary, other);
            if (result.contactCount() == 0) return;

            activatePair(ctx.space, ctx.primary, other);

            const cached = fetchArbiter(ctx.space, ctx.primary, other) orelse return;
            cached.value.syncContacts(result);

            if (handler.begin) |begin_func| {
                if (!begin_func(&cached.value, ctx.space)) return;
            }
            if (handler.preSolve) |pre_func| {
                if (!pre_func(&cached.value, ctx.space)) return;
            }

            ctx.space.arbiters.append(cached.value) catch {};
        }

        fn preStepArbiters(space: *Space, dt: types.cpFloat) void {
            for (space.arbiters.items) |*arb_ref| {
                arb_ref.preStep(dt);
            }
        }

        fn applyCachedArbiterImpulses(space: *Space, dt_coef: types.cpFloat) void {
            for (space.arbiters.items) |*arb_ref| {
                arb_ref.applyCachedImpulse(dt_coef);
            }
        }

        fn resolveArbiters(space: *Space) void {
            for (space.arbiters.items) |*arb_ref| {
                arb_ref.applyImpulse();
            }
        }

        fn runSeparations(space: *Space) void {
            for (space.arbiters.items) |*arb_ref| {
                const handler = selectHandler(space, arb_ref.shape_a, arb_ref.shape_b);
                if (handler.separate) |sep_func| {
                    sep_func(arb_ref, space);
                }
            }
        }

        fn runPostSolve(space: *Space) void {
            for (space.arbiters.items) |*arb_ref| {
                const handler = selectHandler(space, arb_ref.shape_a, arb_ref.shape_b);
                if (handler.postSolve) |post_func| {
                    post_func(arb_ref, space);
                }
            }
        }

        fn runPostSteps(space: *Space) void {
            for (space.post_steps.items) |callback| {
                callback.func(space, callback.data);
            }
            space.post_steps.clearRetainingCapacity();
        }

        fn updateSleepStates(space: *Space) void {
            for (space.bodies.items) |body| {
                if (body.body_type != .dynamic) continue;
                if (body.sleeping) continue;
                const energy = body.kineticEnergy();
                if (energy > space.sleep_energy_threshold) {
                    body.idle_stamp = space.stamp;
                    continue;
                }

                if (space.stamp - body.idle_stamp >= space.sleep_delay) {
                    body.sleeping = true;
                    body.v = vect.cpvzero;
                    body.w = 0.0;
                }
            }
        }

        fn activatePair(space: *Space, a: *shape_base.cpShape, b: *shape_base.cpShape) void {
            space.activateBody(a.body);
            space.activateBody(b.body);
        }

        fn selectHandler(space: *Space, a: *shape_base.cpShape, b: *shape_base.cpShape) Handler {
            if (space.handlers.get(a.collision_type)) |handler| return handler;
            if (space.handlers.get(b.collision_type)) |handler| return handler;
            if (space.wildcard_handlers.get(a.collision_type)) |handler| return handler;
            if (space.wildcard_handlers.get(b.collision_type)) |handler| return handler;
            return space.handler;
        }

        fn makePairKey(a: *const shape_base.cpShape, b: *const shape_base.cpShape) u128 {
            const first = @intFromPtr(a);
            const second = @intFromPtr(b);
            const min_ptr = @min(first, second);
            const max_ptr = @max(first, second);
            return (@as(u128, min_ptr) << 64) | @as(u128, max_ptr);
        }

        fn fetchArbiter(space: *Space, a: *shape_base.cpShape, b: *shape_base.cpShape) ?*CachedArbiter {
            const key = makePairKey(a, b);
            var gop = space.arbiter_cache.getOrPut(key) catch return null;
            if (!gop.found_existing) {
                const pooled = takeArbiter(space, a, b);
                gop.value_ptr.* = .{ .value = pooled, .stamp = space.stamp };
            }
            gop.value_ptr.stamp = space.stamp;
            gop.value_ptr.value.reuse(a, b);
            return gop.value_ptr;
        }

        fn pruneArbiterCache(space: *Space) void {
            var stale_keys: std.ArrayList(u128) = .empty;
            defer stale_keys.deinit(space.allocator);

            var it = space.arbiter_cache.iterator();
            while (it.next()) |entry| {
                if (entry.value_ptr.stamp != space.stamp) {
                    stale_keys.append(space.allocator, entry.key_ptr.*) catch {};
                }
            }

            for (stale_keys.items) |key| {
                if (space.arbiter_cache.remove(key)) |cached| {
                    const handler = selectHandler(space, cached.value.shape_a, cached.value.shape_b);
                    if (handler.separate) |sep_func| {
                        var temp = cached.value;
                        sep_func(&temp, space);
                    }
                    stashArbiter(space, cached.value);
                }
            }
        }

        fn recycleArbiters(space: *Space) void {
            space.arbiters.clearRetainingCapacity();
        }

        fn syncArbiterCache(space: *Space) void {
            for (space.arbiters.items) |arb_ref| {
                const key = makePairKey(arb_ref.shape_a, arb_ref.shape_b);
                if (space.arbiter_cache.getPtr(key)) |cached| {
                    cached.value = arb_ref;
                    cached.stamp = space.stamp;
                }
            }
        }

        fn takeArbiter(space: *Space, shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) arbiter.cpArbiter {
            var idx: usize = 0;
            while (idx < space.arbiter_pool.items.len) : (idx += 1) {
                if (space.arbiter_pool.items[idx].matches(shape_a, shape_b)) {
                    var pooled = space.arbiter_pool.swapRemove(idx);
                    pooled.reuse(shape_a, shape_b);
                    return pooled;
                }
            }
            return arbiter.cpArbiter.init(shape_a, shape_b);
        }

        fn stashArbiter(space: *Space, arb_ref: arbiter.cpArbiter) void {
            space.arbiter_pool.append(arb_ref) catch {};
        }

        const QueryContext = struct {
            space: *Space,
            primary: *shape_base.cpShape,
            static_query: bool,
        };
    };
}
