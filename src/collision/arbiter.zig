const std = @import("std");
const vect = @import("../core/vect.zig");
const types = @import("../core/types.zig");
const shape_base = @import("../shape/shape_base.zig");
const body_mod = shape_base.body_mod;
const collision = @import("collision.zig");

const MAX_BIAS: types.cpFloat = 20.0;

pub const ArbiterContact = struct {
    point: vect.cpVect,
    normal: vect.cpVect,
    distance: types.cpFloat,
    hash: types.cpHashValue,
    r1: vect.cpVect = vect.cpvzero,
    r2: vect.cpVect = vect.cpvzero,
    n_mass: types.cpFloat = 0.0,
    t_mass: types.cpFloat = 0.0,
    bias: types.cpFloat = 0.0,
    jn_acc: types.cpFloat = 0.0,
    jt_acc: types.cpFloat = 0.0,
    jbias_acc: types.cpFloat = 0.0,
    target_vrn: types.cpFloat = 0.0,
};

pub const cpContactPointSet = struct {
    count: usize = 0,
    normal: vect.cpVect = vect.cpvzero,
    points: [types.CP_MAX_CONTACTS_PER_ARBITER]Point = [_]Point{.{}} ** types.CP_MAX_CONTACTS_PER_ARBITER,

    pub const Point = struct {
        pointA: vect.cpVect = vect.cpvzero,
        pointB: vect.cpVect = vect.cpvzero,
        distance: types.cpFloat = 0.0,
    };
};

pub const Thread = struct { next: ?*cpArbiter = null };

pub const cpArbiter = struct {
    shape_a: *shape_base.cpShape,
    shape_b: *shape_base.cpShape,
    collision_id: types.cpCollisionID = 0,
    friction: types.cpFloat = 0.0,
    restitution: types.cpFloat = 0.0,
    surface_velocity: vect.cpVect = vect.cpvzero,
    contacts: std.BoundedArray(ArbiterContact, types.CP_MAX_CONTACTS_PER_ARBITER),
    thread_a: Thread = .{},
    thread_b: Thread = .{},

    pub fn init(shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) cpArbiter {
        return .{
            .shape_a = shape_a,
            .shape_b = shape_b,
            .friction = surfaceFriction(shape_a, shape_b),
            .restitution = types.cpfmax(shape_a.elasticity, shape_b.elasticity),
            .surface_velocity = vect.cpvsub(shape_b.surface_velocity, shape_a.surface_velocity),
            .contacts = std.BoundedArray(ArbiterContact, types.CP_MAX_CONTACTS_PER_ARBITER).init(0) catch unreachable,
            .thread_a = .{},
            .thread_b = .{},
        };
    }

    pub fn reuse(self: *cpArbiter, shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) void {
        self.shape_a = shape_a;
        self.shape_b = shape_b;
        self.friction = surfaceFriction(shape_a, shape_b);
        self.restitution = types.cpfmax(shape_a.elasticity, shape_b.elasticity);
        self.surface_velocity = vect.cpvsub(shape_b.surface_velocity, shape_a.surface_velocity);
        self.thread_a = .{};
        self.thread_b = .{};
    }

    pub fn matches(self: cpArbiter, shape_a: *shape_base.cpShape, shape_b: *shape_base.cpShape) bool {
        return (self.shape_a == shape_a and self.shape_b == shape_b) or
            (self.shape_a == shape_b and self.shape_b == shape_a);
    }

    pub fn clear(self: *cpArbiter) void {
        self.contacts.len = 0;
    }

    pub fn involves(self: cpArbiter, body: *body_mod.cpBody) bool {
        return self.shape_a.body == body or self.shape_b.body == body;
    }

    pub fn threadForBody(self: *cpArbiter, body: *body_mod.cpBody) *Thread {
        std.debug.assert(self.shape_a.body == body or self.shape_b.body == body);
        return if (self.shape_a.body == body) &self.thread_a else &self.thread_b;
    }

    pub fn nextForBody(self: *cpArbiter, body: *body_mod.cpBody) ?*cpArbiter {
        return self.threadForBody(body).next;
    }

    pub fn nextPtrForBody(self: *cpArbiter, body: *body_mod.cpBody) *?*cpArbiter {
        return &self.threadForBody(body).next;
    }

    pub fn setNextForBody(self: *cpArbiter, body: *body_mod.cpBody, next: ?*cpArbiter) void {
        self.threadForBody(body).next = next;
    }

    pub fn otherBody(self: cpArbiter, body: *body_mod.cpBody) *body_mod.cpBody {
        std.debug.assert(self.shape_a.body == body or self.shape_b.body == body);
        return if (self.shape_a.body == body) self.shape_b.body else self.shape_a.body;
    }

    pub fn resetThreads(self: *cpArbiter) void {
        self.thread_a = .{};
        self.thread_b = .{};
    }

    pub fn syncContacts(self: *cpArbiter, result: collision.CollisionResult) void {
        var cached = std.BoundedArray(ArbiterContact, types.CP_MAX_CONTACTS_PER_ARBITER).init(0) catch unreachable;
        for (self.contacts.constSlice()) |contact| {
            cached.append(contact) catch {};
        }

        self.contacts.len = 0;
        self.collision_id = result.id;

        for (result.contacts.constSlice()) |incoming| {
            var matched: ?ArbiterContact = null;
            for (cached.constSlice()) |old| {
                if (old.hash == incoming.hash) {
                    matched = old;
                    break;
                }
            }

            var fresh = ArbiterContact{
                .point = incoming.point,
                .normal = incoming.normal,
                .distance = incoming.distance,
                .hash = incoming.hash,
                .jn_acc = if (matched) |m| m.jn_acc else 0.0,
                .jt_acc = if (matched) |m| m.jt_acc else 0.0,
                .jbias_acc = 0.0,
            };
            fresh.target_vrn = -self.restitution * types.cpfmin(0.0, vect.cpvdot(self.surface_velocity, incoming.normal));
            _ = self.contacts.append(fresh) catch {};
        }
    }

    pub fn contactCount(self: cpArbiter) usize {
        return self.contacts.len;
    }

    pub fn penetrationDepth(self: cpArbiter) types.cpFloat {
        var depth: types.cpFloat = 0.0;
        for (self.contacts.constSlice()) |contact| {
            if (contact.distance < 0.0) {
                depth += -contact.distance;
            }
        }
        return depth;
    }

    pub fn contactPointSet(self: cpArbiter) cpContactPointSet {
        var set = cpContactPointSet{};
        const count = @min(self.contacts.len, types.CP_MAX_CONTACTS_PER_ARBITER);
        set.count = count;
        if (count == 0) return set;

        set.normal = self.contacts.constSlice()[0].normal;
        var i: usize = 0;
        while (i < count) : (i += 1) {
            const contact = self.contacts.constSlice()[i];
            set.points[i] = .{
                .pointA = contact.point,
                .pointB = vect.cpvsub(contact.point, vect.cpvmult(contact.normal, contact.distance)),
                .distance = contact.distance,
            };
        }
        return set;
    }

    pub fn preStep(
        self: *cpArbiter,
        dt: types.cpFloat,
        collision_slop: types.cpFloat,
        bias_coef: types.cpFloat,
    ) void {
        const body_a = self.shape_a.body;
        const body_b = self.shape_b.body;

        for (self.contacts.slice()) |*contact| {
            contact.r1 = vect.cpvsub(contact.point, body_a.p);
            contact.r2 = vect.cpvsub(contact.point, body_b.p);

            const rn1 = vect.cpvcross(contact.r1, contact.normal);
            const rn2 = vect.cpvcross(contact.r2, contact.normal);
            const inv_mass = body_a.m_inv + body_b.m_inv + rn1 * rn1 * body_a.i_inv + rn2 * rn2 * body_b.i_inv;
            contact.n_mass = if (inv_mass > 0.0) 1.0 / inv_mass else 0.0;

            const tangent = vect.cpvperp(contact.normal);
            const rt1 = vect.cpvcross(contact.r1, tangent);
            const rt2 = vect.cpvcross(contact.r2, tangent);
            const inv_tangent = body_a.m_inv + body_b.m_inv + rt1 * rt1 * body_a.i_inv + rt2 * rt2 * body_b.i_inv;
            contact.t_mass = if (inv_tangent > 0.0) 1.0 / inv_tangent else 0.0;

            const penetration = types.cpfmin(0.0, contact.distance + collision_slop);
            var bias = -bias_coef * penetration / dt;
            bias = types.cpfclamp(bias, -MAX_BIAS, MAX_BIAS);
            contact.bias = bias;
            contact.jbias_acc = 0.0;
            contact.target_vrn = -self.restitution * types.cpfmin(0.0, vect.cpvdot(self.surface_velocity, contact.normal));
        }
    }

    pub fn applyCachedImpulse(self: *cpArbiter, _: types.cpFloat) void {
        const body_a = self.shape_a.body;
        const body_b = self.shape_b.body;

        for (self.contacts.constSlice()) |contact| {
            applyVelocityImpulse(body_a, body_b, contact.normal, contact.r1, contact.r2, contact.jn_acc, contact.jt_acc);
        }
    }

    pub fn applyImpulse(self: *cpArbiter) void {
        const body_a = self.shape_a.body;
        const body_b = self.shape_b.body;

        for (self.contacts.slice()) |*contact| {
            const normal = contact.normal;
            const tangent = vect.cpvperp(normal);

            const vr = relativeVelocity(body_a, body_b, contact.r1, contact.r2, self.surface_velocity);
            const vrn = vect.cpvdot(vr, normal);
            const vrt = vect.cpvdot(vr, tangent);

            // Bias impulse to correct penetration drift.
            const vb = relativeBiasVelocity(body_a, body_b, contact.r1, contact.r2);
            const vbn = vect.cpvdot(vb, normal);
            const jbn = (contact.bias - vbn) * contact.n_mass;
            contact.jbias_acc += jbn;
            applyBiasImpulse(body_a, body_b, normal, contact.r1, contact.r2, jbn);

            // Restitution and friction impulses.
            const bounce = contact.target_vrn;
            var jn = (bounce - vrn) * contact.n_mass;
            const jn_old = contact.jn_acc;
            contact.jn_acc = types.cpfmax(jn_old + jn, 0.0);
            jn = contact.jn_acc - jn_old;

            var jt = -vrt * contact.t_mass;
            const jt_max = self.friction * contact.jn_acc;
            const jt_old = contact.jt_acc;
            contact.jt_acc = types.cpfclamp(jt_old + jt, -jt_max, jt_max);
            jt = contact.jt_acc - jt_old;

            applyVelocityImpulse(body_a, body_b, normal, contact.r1, contact.r2, jn, jt);
        }
    }
};

pub fn testArbiterAccumulation() !void {
    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);

    var shape_a = shape_base.cpShape.init(.circle, &body_a);
    var shape_b = shape_base.cpShape.init(.circle, &body_b);

    var arb = cpArbiter.init(&shape_a, &shape_b);
    try std.testing.expectEqual(@as(usize, 0), arb.contactCount());

    var result = collision.CollisionResult.empty();
    result.addContact(.{ .point = vect.cpvzero, .normal = vect.cpv(1.0, 0.0), .distance = -0.25, .hash = 1 });
    result.addContact(.{ .point = vect.cpv(0.0, 1.0), .normal = vect.cpv(0.0, 1.0), .distance = 0.1, .hash = 2 });
    arb.syncContacts(result);

    try std.testing.expectEqual(@as(usize, 2), arb.contactCount());
    try std.testing.expectApproxEqAbs(0.25, arb.penetrationDepth(), 1e-6);

    const set = arb.contactPointSet();
    try std.testing.expectEqual(@as(usize, 2), set.count);
    try std.testing.expectApproxEqAbs(-0.25, set.points[0].distance, 1e-6);
    try std.testing.expectApproxEqAbs(0.25, set.points[0].pointB.x, 1e-6);

    arb.clear();
    try std.testing.expectEqual(@as(usize, 0), arb.contactCount());
}

fn surfaceFriction(a: *shape_base.cpShape, b: *shape_base.cpShape) types.cpFloat {
    return std.math.sqrt(a.friction * b.friction);
}

fn relativeVelocity(
    a: *shape_base.body_mod.cpBody,
    b: *shape_base.body_mod.cpBody,
    r1: vect.cpVect,
    r2: vect.cpVect,
    surface_velocity: vect.cpVect,
) vect.cpVect {
    const va = vect.cpvadd(a.v, vect.cpvmult(vect.cpvperp(r1), a.w));
    const vb = vect.cpvadd(b.v, vect.cpvmult(vect.cpvperp(r2), b.w));
    return vect.cpvsub(vect.cpvsub(vb, va), surface_velocity);
}

fn relativeBiasVelocity(
    a: *shape_base.body_mod.cpBody,
    b: *shape_base.body_mod.cpBody,
    r1: vect.cpVect,
    r2: vect.cpVect,
) vect.cpVect {
    const va = vect.cpvadd(a.v_bias, vect.cpvmult(vect.cpvperp(r1), a.w_bias));
    const vb = vect.cpvadd(b.v_bias, vect.cpvmult(vect.cpvperp(r2), b.w_bias));
    return vect.cpvsub(vb, va);
}

fn applyVelocityImpulse(
    body_a: *shape_base.body_mod.cpBody,
    body_b: *shape_base.body_mod.cpBody,
    normal: vect.cpVect,
    r1: vect.cpVect,
    r2: vect.cpVect,
    jn: types.cpFloat,
    jt: types.cpFloat,
) void {
    const tangent = vect.cpvperp(normal);
    const impulse = vect.cpvadd(vect.cpvmult(normal, jn), vect.cpvmult(tangent, jt));
    body_a.v = vect.cpvsub(body_a.v, vect.cpvmult(impulse, body_a.m_inv));
    body_a.w -= body_a.i_inv * vect.cpvcross(r1, impulse);
    body_b.v = vect.cpvadd(body_b.v, vect.cpvmult(impulse, body_b.m_inv));
    body_b.w += body_b.i_inv * vect.cpvcross(r2, impulse);
}

fn applyBiasImpulse(
    body_a: *shape_base.body_mod.cpBody,
    body_b: *shape_base.body_mod.cpBody,
    normal: vect.cpVect,
    r1: vect.cpVect,
    r2: vect.cpVect,
    jn: types.cpFloat,
) void {
    const impulse = vect.cpvmult(normal, jn);
    body_a.v_bias = vect.cpvsub(body_a.v_bias, vect.cpvmult(impulse, body_a.m_inv));
    body_a.w_bias -= body_a.i_inv * vect.cpvcross(r1, impulse);
    body_b.v_bias = vect.cpvadd(body_b.v_bias, vect.cpvmult(impulse, body_b.m_inv));
    body_b.w_bias += body_b.i_inv * vect.cpvcross(r2, impulse);
}

comptime {
    std.testing.refAllDecls(@This());
}
