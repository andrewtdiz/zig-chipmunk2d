const std = @import("std");
const vect = @import("../core/vect.zig");
const types = @import("../core/types.zig");
const shape_base = @import("../shape/shape_base.zig");
const collision = @import("collision.zig");

pub const ContactPointState = struct {
    contact: collision.Contact = .{ .point = vect.cpvzero, .normal = vect.cpv(1.0, 0.0), .distance = 0.0 },
    r_a: vect.cpVect = vect.cpvzero,
    r_b: vect.cpVect = vect.cpvzero,
    normal_impulse: types.cpFloat = 0.0,
    tangent_impulse: types.cpFloat = 0.0,
    bias: types.cpFloat = 0.0,
};

pub const ContactBuffer = struct {
    points: std.BoundedArray(ContactPointState, collision.max_contacts),

    pub fn init() ContactBuffer {
        return .{ .points = std.BoundedArray(ContactPointState, collision.max_contacts).init(0) catch unreachable };
    }

    pub fn reset(self: *ContactBuffer) void {
        self.points.len = 0;
    }
};

pub const State = enum { first, normal, ignore };

pub const cpArbiter = struct {
    shape_a: *shape_base.cpShape,
    shape_b: *shape_base.cpShape,
    buffer: *ContactBuffer,
    friction: types.cpFloat,
    elasticity: types.cpFloat,
    surface_velocity: vect.cpVect,
    stamp: usize = 0,
    state: State = .first,

    pub fn init(shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape, buffer: *ContactBuffer) cpArbiter {
        var arb = cpArbiter{
            .shape_a = shape_a,
            .shape_b = shape_b,
            .buffer = buffer,
            .friction = 0.0,
            .elasticity = 0.0,
            .surface_velocity = vect.cpvzero,
        };
        arb.updateShapes(shape_a, shape_b);
        buffer.reset();
        return arb;
    }

    pub fn clear(self: *cpArbiter) void {
        self.buffer.reset();
    }

    pub fn updateShapes(self: *cpArbiter, shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) void {
        self.shape_a = shape_a;
        self.shape_b = shape_b;
        self.friction = shape_a.friction * shape_b.friction;
        self.elasticity = shape_a.elasticity * shape_b.elasticity;
        self.surface_velocity = vect.cpvsub(shape_b.surface_velocity, shape_a.surface_velocity);
    }

    pub fn updateContacts(self: *cpArbiter, result: collision.CollisionResult) void {
        var previous: [collision.max_contacts]ContactPointState = undefined;
        var used: [collision.max_contacts]bool = undefined;
        @memset(&used, false);
        const old = self.buffer.points.constSlice();
        var idx: usize = 0;
        while (idx < old.len) : (idx += 1) {
            previous[idx] = old[idx];
        }

        self.buffer.reset();
        for (result.contacts.constSlice()) |contact| {
            var state = ContactPointState{ .contact = contact };
            state.r_a = vect.cpvsub(contact.point, self.shape_a.body.p);
            state.r_b = vect.cpvsub(contact.point, self.shape_b.body.p);

            var best_index: ?usize = null;
            var best_metric = types.CP_INFINITY;
            var search: usize = 0;
            while (search < old.len) : (search += 1) {
                if (used[search]) continue;
                const delta = vect.cpvsub(contact.point, previous[search].contact.point);
                const metric = vect.cpvdot(delta, delta);
                if (metric < best_metric) {
                    best_metric = metric;
                    best_index = search;
                }
            }

            if (best_index) |match_index| {
                used[match_index] = true;
                state.normal_impulse = previous[match_index].normal_impulse;
                state.tangent_impulse = previous[match_index].tangent_impulse;
            }

            _ = self.buffer.points.append(state) catch {};
        }
    }

    pub fn addContact(self: *cpArbiter, contact: collision.Contact) void {
        var state = ContactPointState{ .contact = contact };
        state.r_a = vect.cpvsub(contact.point, self.shape_a.body.p);
        state.r_b = vect.cpvsub(contact.point, self.shape_b.body.p);
        _ = self.buffer.points.append(state) catch {};
    }

    pub fn contactCount(self: cpArbiter) usize {
        return self.buffer.points.len;
    }

    pub fn contacts(self: *cpArbiter) []ContactPointState {
        return self.buffer.points.slice();
    }

    pub fn penetrationDepth(self: cpArbiter) types.cpFloat {
        var depth: types.cpFloat = 0.0;
        for (self.buffer.points.constSlice()) |state| {
            if (state.contact.distance < 0.0) {
                depth += -state.contact.distance;
            }
        }
        return depth;
    }

    pub fn preStep(self: *cpArbiter, dt: types.cpFloat) void {
        const bias_coef = 0.2;
        for (self.buffer.points.slice()) |*state| {
            const slop = 0.01;
            const pen = state.contact.distance + slop;
            if (pen < 0.0 and dt != 0.0) {
                state.bias = -bias_coef * pen / dt;
            } else {
                state.bias = 0.0;
            }
        }
    }

    pub fn applyCachedImpulse(self: *cpArbiter, dt_coef: types.cpFloat) void {
        for (self.buffer.points.slice()) |*state| {
            const old_normal = state.normal_impulse;
            const old_tangent = state.tangent_impulse;
            state.normal_impulse *= dt_coef;
            state.tangent_impulse *= dt_coef;
            const delta_n = state.normal_impulse - old_normal;
            const delta_t = state.tangent_impulse - old_tangent;
            const n = state.contact.normal;
            const t = vect.cpvperp(n);
            const impulse = vect.cpvadd(vect.cpvmult(n, delta_n), vect.cpvmult(t, delta_t));
            applyImpulsePair(self.shape_a.body, self.shape_b.body, state.r_a, state.r_b, impulse);
        }
    }

    pub fn applyImpulse(self: *cpArbiter, dt: types.cpFloat) void {
        for (self.buffer.points.slice()) |*state| {
            solveNormal(self, state, dt);
            solveTangent(self, state, dt);
        }
    }
};

fn solveNormal(self: *cpArbiter, state: *ContactPointState, dt: types.cpFloat) void {
    const n = state.contact.normal;
    const mass = normalMass(self, state, n);
    if (mass == 0.0) return;

    const rel = relativeVelocity(self.shape_a.body, self.shape_b.body, state.r_a, state.r_b);
    const vrn = vect.cpvdot(rel, n) - vect.cpvdot(self.surface_velocity, n);
    const bias = state.bias;
    const jn = -(vrn + bias) * mass;
    const j_old = state.normal_impulse;
    const max_impulse = types.cpfmax(0.0, self.shape_a.body.m_inv + self.shape_b.body.m_inv) * dt;
    state.normal_impulse = types.cpfmax(j_old + jn, 0.0);
    if (max_impulse > 0.0) {
        state.normal_impulse = types.cpfmin(state.normal_impulse, max_impulse);
    }
    const delta = state.normal_impulse - j_old;
    if (delta == 0.0) return;
    const impulse = vect.cpvmult(n, delta);
    applyImpulsePair(self.shape_a.body, self.shape_b.body, state.r_a, state.r_b, impulse);
}

fn solveTangent(self: *cpArbiter, state: *ContactPointState, dt: types.cpFloat) void {
    _ = dt;
    const t = vect.cpvperp(state.contact.normal);
    const mass = normalMass(self, state, t);
    if (mass == 0.0) return;

    const rel = relativeVelocity(self.shape_a.body, self.shape_b.body, state.r_a, state.r_b);
    const vrt = vect.cpvdot(rel, t) - vect.cpvdot(self.surface_velocity, t);
    const jt = -vrt * mass;
    const max_friction = self.friction * state.normal_impulse;
    const old = state.tangent_impulse;
    state.tangent_impulse = types.cpfclamp(old + jt, -max_friction, max_friction);
    const delta = state.tangent_impulse - old;
    if (delta == 0.0) return;
    const impulse = vect.cpvmult(t, delta);
    applyImpulsePair(self.shape_a.body, self.shape_b.body, state.r_a, state.r_b, impulse);
}

fn normalMass(self: *cpArbiter, state: *ContactPointState, axis: vect.cpVect) types.cpFloat {
    const r1 = vect.cpvcross(state.r_a, axis);
    const r2 = vect.cpvcross(state.r_b, axis);
    const denom = self.shape_a.body.m_inv + self.shape_b.body.m_inv + self.shape_a.body.i_inv * r1 * r1 + self.shape_b.body.i_inv * r2 * r2;
    return if (denom == 0.0) 0.0 else 1.0 / denom;
}

fn relativeVelocity(a: *shape_base.body_mod.cpBody, b: *shape_base.body_mod.cpBody, r_a: vect.cpVect, r_b: vect.cpVect) vect.cpVect {
    const va = vect.cpvadd(a.v, vect.cpvmult(vect.cpvperp(r_a), a.w));
    const vb = vect.cpvadd(b.v, vect.cpvmult(vect.cpvperp(r_b), b.w));
    return vect.cpvsub(vb, va);
}

fn applyImpulsePair(a: *shape_base.body_mod.cpBody, b: *shape_base.body_mod.cpBody, r_a: vect.cpVect, r_b: vect.cpVect, impulse: vect.cpVect) void {
    const point_a = vect.cpvadd(a.p, r_a);
    const point_b = vect.cpvadd(b.p, r_b);
    a.applyImpulseAtWorldPoint(vect.cpvneg(impulse), point_a);
    b.applyImpulseAtWorldPoint(impulse, point_b);
}

pub fn testArbiterAccumulation() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    var shape_a = shape_base.cpShape.init(.circle, &body_a);
    var shape_b = shape_base.cpShape.init(.circle, &body_b);

    var buffer_store = ContactBuffer.init();
    var arbiter_instance = cpArbiter.init(&shape_a, &shape_b, &buffer_store);
    var arbiter = &arbiter_instance;
    try std.testing.expectEqual(@as(usize, 0), arbiter.contactCount());

    arbiter.addContact(.{ .point = vect.cpvzero, .normal = vect.cpv(1.0, 0.0), .distance = -0.25 });
    arbiter.addContact(.{ .point = vect.cpv(0.0, 1.0), .normal = vect.cpv(0.0, 1.0), .distance = 0.1 });

    try std.testing.expectEqual(@as(usize, 2), arbiter.contactCount());
    try std.testing.expectApproxEqAbs(0.25, arbiter.penetrationDepth(), 1e-6);

    arbiter.clear();
    try std.testing.expectEqual(@as(usize, 0), arbiter.contactCount());
}

test "arbiter preserves cached impulses" {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    var shape_a = shape_base.cpShape.init(.circle, &body_a);
    var shape_b = shape_base.cpShape.init(.circle, &body_b);

    var buffer_store = ContactBuffer.init();
    var arbiter_instance = cpArbiter.init(&shape_a, &shape_b, &buffer_store);
    var arbiter = &arbiter_instance;

    const contact = collision.Contact{ .point = vect.cpv(0.0, 0.0), .normal = vect.cpv(1.0, 0.0), .distance = -0.1 };
    arbiter.addContact(contact);
    arbiter.buffer.points.slice()[0].normal_impulse = 5.0;

    var next = collision.CollisionResult.empty();
    next.addContact(.{ .point = vect.cpv(0.001, 0.0), .normal = vect.cpv(1.0, 0.0), .distance = -0.1 });
    arbiter.updateContacts(next);

    try std.testing.expectApproxEqAbs(5.0, arbiter.buffer.points.slice()[0].normal_impulse, 1e-6);
}

comptime {
    std.testing.refAllDecls(@This());
}
