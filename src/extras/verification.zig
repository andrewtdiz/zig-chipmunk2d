const std = @import("std");
const builtin = @import("builtin");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const body_mod = @import("../space/body.zig");
const circle = @import("../shape/circle.zig");
const space_mod = @import("../space/space.zig");
const hasty_space = @import("./hasty_space.zig");

const c = if (builtin.is_test)
    @cImport({
        @cDefine("CP_USE_CGPOINTS", "0");
        @cInclude("chipmunk/chipmunk.h");
    })
else
    struct {};

const SimulationSnapshot = struct {
    positions: []vect.cpVect,
    velocities: []vect.cpVect,
};

const BodyInit = struct {
    mass: types.cpFloat,
    radius: types.cpFloat,
    position: vect.cpVect,
    velocity: vect.cpVect,
};

const ParityScenario = struct {
    gravity: vect.cpVect,
    damping: types.cpFloat,
    dt: types.cpFloat,
    steps: usize,
    iterations: usize,
    friction: types.cpFloat,
    elasticity: types.cpFloat,
    initial: []const BodyInit,
};

const tolerance: types.cpFloat = 1e-4;

fn baseScenario(arena: *std.heap.ArenaAllocator) ParityScenario {
    const allocator = arena.allocator();
    const bodies = allocator.alloc(BodyInit, 2) catch unreachable;
    bodies[0] = .{
        .mass = 1.0,
        .radius = 0.5,
        .position = vect.cpv(-1.25, 0.2),
        .velocity = vect.cpv(4.0, 0.0),
    };
    bodies[1] = .{
        .mass = 1.0,
        .radius = 0.5,
        .position = vect.cpv(1.25, -0.1),
        .velocity = vect.cpv(-4.0, 0.2),
    };

    return .{
        .gravity = vect.cpv(0.0, -1.5),
        .damping = 0.98,
        .dt = 1.0 / 60.0,
        .steps = 60,
        .iterations = 15,
        .friction = 0.9,
        .elasticity = 0.1,
        .initial = bodies,
    };
}

fn snapshotFromBodies(allocator: std.mem.Allocator, bodies: []const body_mod.cpBody) !SimulationSnapshot {
    const positions = try allocator.alloc(vect.cpVect, bodies.len);
    const velocities = try allocator.alloc(vect.cpVect, bodies.len);
    for (bodies, 0..) |body, index| {
        positions[index] = body.p;
        velocities[index] = body.v;
    }
    return .{ .positions = positions, .velocities = velocities };
}

fn simulateZigSpace(allocator: std.mem.Allocator, scenario: ParityScenario) !SimulationSnapshot {
    var space = space_mod.cpSpace.init(allocator);
    space.gravity = scenario.gravity;
    space.damping = scenario.damping;
    space.iterations = scenario.iterations;

    var bodies = try allocator.alloc(body_mod.cpBody, scenario.initial.len);
    var shapes = try allocator.alloc(circle.cpCircleShape, scenario.initial.len);
    defer allocator.free(shapes);
    defer allocator.free(bodies);
    defer space.deinit();

    for (scenario.initial, 0..) |init, index| {
        bodies[index] = body_mod.cpBody.init(init.mass, body_mod.cpMomentForCircle(init.mass, 0.0, init.radius, vect.cpvzero));
        bodies[index].setPosition(init.position);
        bodies[index].setVelocity(init.velocity);

        shapes[index] = circle.cpCircleShape.init(&bodies[index], init.radius, vect.cpvzero);
        shapes[index].base.friction = scenario.friction;
        shapes[index].base.elasticity = scenario.elasticity;

        try space.addBody(&bodies[index]);
        try space.addShape(&shapes[index].base);
    }

    var step: usize = 0;
    while (step < scenario.steps) : (step += 1) {
        space.step(scenario.dt);
    }

    return snapshotFromBodies(allocator, bodies);
}

fn toCVect(value: vect.cpVect) c.cpVect {
    return c.cpv(value.x, value.y);
}

fn simulateCReference(allocator: std.mem.Allocator, scenario: ParityScenario) !SimulationSnapshot {
    const space = c.cpSpaceNew();
    defer c.cpSpaceFree(space);

    c.cpSpaceSetGravity(space, toCVect(scenario.gravity));
    c.cpSpaceSetDamping(space, scenario.damping);
    c.cpSpaceSetIterations(space, @intCast(scenario.iterations));

    var bodies = try allocator.alloc(*c.cpBody, scenario.initial.len);
    defer allocator.free(bodies);
    var shapes = try allocator.alloc(*c.cpShape, scenario.initial.len);
    defer allocator.free(shapes);

    for (scenario.initial, 0..) |init, index| {
        const body = c.cpBodyNew(init.mass, c.cpMomentForCircle(init.mass, 0.0, init.radius, c.cpvzero));
        bodies[index] = body;
        c.cpBodySetPosition(body, toCVect(init.position));
        c.cpBodySetVelocity(body, toCVect(init.velocity));

        const circle_shape = c.cpCircleShapeNew(body, init.radius, c.cpvzero);
        shapes[index] = @ptrCast(circle_shape);
        c.cpShapeSetFriction(circle_shape, scenario.friction);
        c.cpShapeSetElasticity(circle_shape, scenario.elasticity);

        _ = c.cpSpaceAddBody(space, body);
        _ = c.cpSpaceAddShape(space, circle_shape);
    }

    var step: usize = 0;
    while (step < scenario.steps) : (step += 1) {
        c.cpSpaceStep(space, scenario.dt);
    }

    var snapshot = SimulationSnapshot{
        .positions = try allocator.alloc(vect.cpVect, scenario.initial.len),
        .velocities = try allocator.alloc(vect.cpVect, scenario.initial.len),
    };

    for (bodies, 0..) |body, index| {
        const pos = c.cpBodyGetPosition(body);
        const vel = c.cpBodyGetVelocity(body);
        snapshot.positions[index] = vect.cpv(pos.x, pos.y);
        snapshot.velocities[index] = vect.cpv(vel.x, vel.y);
        c.cpSpaceRemoveShape(space, shapes[index]);
        c.cpSpaceRemoveBody(space, body);
        c.cpShapeFree(shapes[index]);
        c.cpBodyFree(body);
    }

    return snapshot;
}

fn simulateHastySpace(allocator: std.mem.Allocator, scenario: ParityScenario, threads: usize) !SimulationSnapshot {
    var hasty = hasty_space.cpHastySpace.init(allocator);
    try hasty.setThreads(threads);
    hasty.space.gravity = scenario.gravity;
    hasty.space.damping = scenario.damping;
    hasty.space.iterations = scenario.iterations;

    var bodies = try allocator.alloc(body_mod.cpBody, scenario.initial.len);
    var shapes = try allocator.alloc(circle.cpCircleShape, scenario.initial.len);
    defer allocator.free(shapes);
    defer allocator.free(bodies);
    defer hasty.deinit();

    for (scenario.initial, 0..) |init, index| {
        bodies[index] = body_mod.cpBody.init(init.mass, body_mod.cpMomentForCircle(init.mass, 0.0, init.radius, vect.cpvzero));
        bodies[index].setPosition(init.position);
        bodies[index].setVelocity(init.velocity);

        shapes[index] = circle.cpCircleShape.init(&bodies[index], init.radius, vect.cpvzero);
        shapes[index].base.friction = scenario.friction;
        shapes[index].base.elasticity = scenario.elasticity;

        try hasty.addBody(&bodies[index]);
        try hasty.addShape(&shapes[index].base);
    }

    var step: usize = 0;
    while (step < scenario.steps) : (step += 1) {
        hasty.step(scenario.dt);
    }

    return snapshotFromBodies(allocator, bodies);
}

fn assertClose(actual: SimulationSnapshot, expected: SimulationSnapshot) !void {
    for (actual.positions, 0..) |pos, index| {
        try std.testing.expectApproxEqAbs(expected.positions[index].x, pos.x, tolerance);
        try std.testing.expectApproxEqAbs(expected.positions[index].y, pos.y, tolerance);
    }
    for (actual.velocities, 0..) |vel, index| {
        try std.testing.expectApproxEqAbs(expected.velocities[index].x, vel.x, tolerance);
        try std.testing.expectApproxEqAbs(expected.velocities[index].y, vel.y, tolerance);
    }
}

test "Chipmunk C parity matches Zig space for head-on exchange" {
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();

    const scenario = baseScenario(&arena);
    const zig = try simulateZigSpace(arena.allocator(), scenario);
    defer {
        arena.allocator().free(zig.positions);
        arena.allocator().free(zig.velocities);
    }
    const reference = try simulateCReference(arena.allocator(), scenario);
    defer {
        arena.allocator().free(reference.positions);
        arena.allocator().free(reference.velocities);
    }

    try assertClose(zig, reference);
}

test "Randomized parity stays within tolerance" {
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();

    var prng = std.rand.DefaultPrng.init(0xdecafbad);
    var iteration: usize = 0;
    while (iteration < 5) : (iteration += 1) {
        const rng = prng.random();
        const offset = rng.float(types.cpFloat) * 0.4;
        const y_offset = (rng.float(types.cpFloat) - 0.5) * 0.3;
        const speed = 2.5 + rng.float(types.cpFloat) * 2.0;

        const scenario = blk: {
            const bodies = arena.allocator().alloc(BodyInit, 2) catch unreachable;
            bodies[0] = .{
                .mass = 1.0,
                .radius = 0.4,
                .position = vect.cpv(-1.2 - offset, y_offset),
                .velocity = vect.cpv(speed, 0.05),
            };
            bodies[1] = .{
                .mass = 1.0,
                .radius = 0.4,
                .position = vect.cpv(1.2 + offset, -y_offset),
                .velocity = vect.cpv(-speed, -0.05),
            };
            break :blk .{
                .gravity = vect.cpv(0.0, -1.0),
                .damping = 0.97,
                .dt = 1.0 / 120.0,
                .steps = 90,
                .iterations = 20,
                .friction = 0.8,
                .elasticity = 0.05,
                .initial = bodies,
            };
        };

        const zig = try simulateZigSpace(arena.allocator(), scenario);
        defer {
            arena.allocator().free(zig.positions);
            arena.allocator().free(zig.velocities);
        }
        const reference = try simulateCReference(arena.allocator(), scenario);
        defer {
            arena.allocator().free(reference.positions);
            arena.allocator().free(reference.velocities);
        }

        try assertClose(zig, reference);
    }
}

test "Zig space runs are deterministic" {
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();

    const scenario = baseScenario(&arena);
    const first = try simulateZigSpace(arena.allocator(), scenario);
    defer {
        arena.allocator().free(first.positions);
        arena.allocator().free(first.velocities);
    }
    const second = try simulateZigSpace(arena.allocator(), scenario);
    defer {
        arena.allocator().free(second.positions);
        arena.allocator().free(second.velocities);
    }

    try assertClose(first, second);
}

test "cpHastySpace stays in sync with cpSpace" {
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();

    var scenario = baseScenario(&arena);
    scenario.steps = 45;
    scenario.iterations = 10;

    const zig = try simulateZigSpace(arena.allocator(), scenario);
    defer {
        arena.allocator().free(zig.positions);
        arena.allocator().free(zig.velocities);
    }
    const hasty = try simulateHastySpace(arena.allocator(), scenario, 2);
    defer {
        arena.allocator().free(hasty.positions);
        arena.allocator().free(hasty.velocities);
    }

    try assertClose(hasty, zig);
}

test "cpHastySpace determinism across runs" {
    var arena = std.heap.ArenaAllocator.init(std.testing.allocator);
    defer arena.deinit();

    var scenario = baseScenario(&arena);
    scenario.steps = 30;

    const first = try simulateHastySpace(arena.allocator(), scenario, 3);
    defer {
        arena.allocator().free(first.positions);
        arena.allocator().free(first.velocities);
    }
    const second = try simulateHastySpace(arena.allocator(), scenario, 3);
    defer {
        arena.allocator().free(second.positions);
        arena.allocator().free(second.velocities);
    }

    try assertClose(first, second);
}
