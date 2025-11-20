const std = @import("std");
const types = @import("../core/types.zig");
const bb = @import("../core/bb.zig");
const vect = @import("../core/vect.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("../collision/collision.zig");
const arbiter = @import("../collision/arbiter.zig");

pub fn makeStepper(comptime Space: type, comptime Helpers: type) type {
    return struct {
        const Handler = @TypeOf(@as(*Space, undefined).handler);
        const CachedArbiter = @TypeOf(@as(*Space, undefined).arbiter_cache).Value;

        pub fn step(space: *Space, dt: types.cpFloat) void {
            space.stamp +%= 1;
            const dt_coef: types.cpFloat = if (dt != 0.0) dt else 1.0;
            updateVelocities(space, dt);
            runConstraintCallback(space, dt, dt_coef, .preStep);
            runConstraintCallback(space, dt, dt_coef, .applyCachedImpulse);
            recycleArbiters(space);
            cacheAllShapes(space);
            space.dynamic_index.reindex() catch {};
            broadPhase(space);
            preStepArbiters(space, dt);
            applyCachedArbiterImpulses(space, dt_coef);

            var iteration: usize = 0;
            while (iteration < space.iterations) : (iteration += 1) {
                runConstraintCallback(space, dt, dt_coef, .applyImpulse);
                resolveArbiters(space);
            }

            runPostSolve(space);
            runSeparations(space);
            integratePositions(space, dt);
            updateSleepStates(space);
            runConstraintCallback(space, dt, dt_coef, .postStep);
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
            for (space.bodies.items) |body| {
                if (body.sleeping) continue;
                body.updateVelocity(space.gravity, space.damping, dt);
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

        fn makePairKey(a: *const shape_base.cpShape, b: *const shape_base.cpShape) u128 {
            const first = @intFromPtr(a);
            const second = @intFromPtr(b);
            const min_ptr = @min(first, second);
            const max_ptr = @max(first, second);
            return (@as(u128, min_ptr) << 64) | @as(u128, max_ptr);
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
