const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("../collision/collision.zig");
const arbiter = @import("../collision/arbiter.zig");

pub fn makeStepper(comptime Space: type, comptime Helpers: type) type {
    return struct {
        const Handler = @TypeOf(@as(*Space, undefined).handler);

        pub fn step(space: *Space, dt: types.cpFloat) void {
            space.stamp +%= 1;
            const dt_coef: types.cpFloat = if (dt != 0.0) dt else 1.0;
            updateVelocities(space, dt);
            runConstraintCallback(space, dt, dt_coef, .preStep);
            runConstraintCallback(space, dt, dt_coef, .applyCachedImpulse);
            cacheAllShapes(space);
            space.dynamic_index.reindex() catch {};
            broadPhase(space);

            var iteration: usize = 0;
            while (iteration < space.iterations) : (iteration += 1) {
                runConstraintCallback(space, dt, dt_coef, .applyImpulse);
                resolveArbiters(space);
            }

            runPostSolve(space);
            integratePositions(space, dt);
            updateSleepStates(space);
            runConstraintCallback(space, dt, dt_coef, .postStep);
            runPostSteps(space);
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
            space.arbiters.clearRetainingCapacity();
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

            const arb_ptr = fetchArbiter(ctx.space, ctx.primary, other) orelse {
                var fallback = arbiter.cpArbiter.init(ctx.primary, other);
                fallback.collision_id = result.id;
                for (result.contacts.constSlice()) |contact| {
                    fallback.addContact(contact);
                }
                if (handler.begin) |begin_func| {
                    if (!begin_func(&fallback, ctx.space)) return;
                }
                if (handler.preSolve) |pre_func| {
                    if (!pre_func(&fallback, ctx.space)) return;
                }
                ctx.space.arbiters.append(fallback) catch return;
                return;
            };

            arb_ptr.collision_id = result.id;
            for (result.contacts.constSlice()) |contact| {
                arb_ptr.addContact(contact);
            }

            if (handler.begin) |begin_func| {
                if (!begin_func(arb_ptr, ctx.space)) return;
            }
            if (handler.preSolve) |pre_func| {
                if (!pre_func(arb_ptr, ctx.space)) return;
            }

            ctx.space.arbiters.append(arb_ptr.*) catch return;
        }

        fn resolveArbiters(space: *Space) void {
            for (space.arbiters.items) |*arb_ref| {
                const shape_a = arb_ref.shape_a;
                const shape_b = arb_ref.shape_b;
                const body_a = shape_a.body;
                const body_b = shape_b.body;

                for (arb_ref.contacts.constSlice()) |contact| {
                    if (contact.distance >= 0.0) continue;
                    const total_inv = body_a.m_inv + body_b.m_inv;
                    if (total_inv == 0.0) continue;

                    const penetration = -contact.distance;
                    const correction = vect.cpvmult(contact.normal, penetration);
                    body_a.p = vect.cpvsub(body_a.p, vect.cpvmult(correction, body_a.m_inv / total_inv));
                    body_b.p = vect.cpvadd(body_b.p, vect.cpvmult(correction, body_b.m_inv / total_inv));

                    const relative = vect.cpvsub(body_b.v, body_a.v);
                    const vel_normal = vect.cpvdot(relative, contact.normal);
                    if (vel_normal > 0.0) continue;
                    const elasticity = types.cpfmax(shape_a.elasticity, shape_b.elasticity);
                    const impulse = -(1.0 + elasticity) * vel_normal / total_inv;
                    const impulse_vec = vect.cpvmult(contact.normal, impulse);
                    body_a.v = vect.cpvsub(body_a.v, vect.cpvmult(impulse_vec, body_a.m_inv));
                    body_b.v = vect.cpvadd(body_b.v, vect.cpvmult(impulse_vec, body_b.m_inv));
                }

                if (space.handler.separate) |sep_func| {
                    sep_func(arb_ref, space);
                }
            }
        }

        fn runPostSolve(space: *Space) void {
            if (space.handler.postSolve) |post_func| {
                for (space.arbiters.items) |*arb_ref| {
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

        fn fetchArbiter(space: *Space, a: *shape_base.cpShape, b: *shape_base.cpShape) ?*arbiter.cpArbiter {
            const key = makePairKey(a, b);
            var gop = space.arbiter_cache.getOrPut(key) catch return null;
            if (!gop.found_existing) {
                gop.value_ptr.* = .{ .value = arbiter.cpArbiter.init(a, b), .stamp = space.stamp };
            }
            gop.value_ptr.stamp = space.stamp;
            gop.value_ptr.value.shape_a = a;
            gop.value_ptr.value.shape_b = b;
            gop.value_ptr.value.clear();
            return &gop.value_ptr.value;
        }

        fn pruneArbiterCache(space: *Space) void {
            var stale_keys = std.ArrayList(u128).init(space.allocator);
            defer stale_keys.deinit();

            var it = space.arbiter_cache.iterator();
            while (it.next()) |entry| {
                if (entry.value_ptr.stamp != space.stamp) {
                    stale_keys.append(entry.key_ptr.*) catch {};
                }
            }

            for (stale_keys.items) |key| {
                _ = space.arbiter_cache.remove(key);
            }
        }

        fn makePairKey(a: *const shape_base.cpShape, b: *const shape_base.cpShape) u128 {
            const first = @intFromPtr(a);
            const second = @intFromPtr(b);
            const min_ptr = @min(first, second);
            const max_ptr = @max(first, second);
            return (@as(u128, min_ptr) << 64) | @as(u128, max_ptr);
        }

        const QueryContext = struct {
            space: *Space,
            primary: *shape_base.cpShape,
            static_query: bool,
        };
    };
}
