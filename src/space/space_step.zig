const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const bb = @import("../core/bb.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("../collision/collision.zig");
const arbiter = @import("../collision/arbiter.zig");

pub fn makeStepper(comptime Space: type, comptime Helpers: type) type {
    return struct {
        pub fn step(space: *Space, dt: types.cpFloat) void {
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

            integratePositions(space, dt);
            runConstraintCallback(space, dt, dt_coef, .postStep);
            runPostSteps(space);
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
                body.updateVelocity(space.gravity, space.damping, dt);
            }
        }

        fn integratePositions(space: *Space, dt: types.cpFloat) void {
            for (space.bodies.items) |body| {
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
            if (shape_base.cpShapeFilter.reject(ctx.primary.filter, other.filter)) return;
            const result = collision.collide(ctx.primary, other);
            if (result.contactCount() == 0) return;

            var new_arb = arbiter.cpArbiter.init(ctx.primary, other);
            for (result.contacts.constSlice()) |contact| {
                new_arb.addContact(contact);
            }

            if (ctx.space.handler.begin) |begin_func| {
                if (!begin_func(&new_arb, ctx.space)) return;
            }
            if (ctx.space.handler.preSolve) |pre_func| {
                if (!pre_func(&new_arb, ctx.space)) return;
            }

            ctx.space.arbiters.append(new_arb) catch return;
            if (ctx.space.handler.postSolve) |post_func| {
                post_func(&new_arb, ctx.space);
            }
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

        fn runPostSteps(space: *Space) void {
            for (space.post_steps.items) |callback| {
                callback.func(space, callback.data);
            }
            space.post_steps.clearRetainingCapacity();
        }

        const QueryContext = struct {
            space: *Space,
            primary: *shape_base.cpShape,
            static_query: bool,
        };
    };
}
