const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");
const space_mod = @import("../space/space.zig");
const shape_base = @import("../shape/shape_base.zig");
const constraint_base = @import("../constraint/constraint_base.zig");
const circle = @import("../shape/circle.zig");
const segment = @import("../shape/segment.zig");
const poly = @import("../shape/poly.zig");

pub const cpSpaceDebugColor = struct {
    r: types.cpFloat,
    g: types.cpFloat,
    b: types.cpFloat,
    a: types.cpFloat,

    pub fn rgba(r: types.cpFloat, g: types.cpFloat, b: types.cpFloat, a: types.cpFloat) cpSpaceDebugColor {
        return .{ .r = r, .g = g, .b = b, .a = a };
    }

    pub fn white() cpSpaceDebugColor {
        return rgba(1.0, 1.0, 1.0, 1.0);
    }

    pub fn gray() cpSpaceDebugColor {
        return rgba(0.6, 0.6, 0.6, 1.0);
    }

    pub fn red() cpSpaceDebugColor {
        return rgba(1.0, 0.2, 0.2, 1.0);
    }
};

pub const cpSpaceDebugFlags = struct {
    draw_shapes: bool = true,
    draw_constraints: bool = true,
    draw_collision_points: bool = true,
};

pub const cpSpaceDebugDrawOptions = struct {
    drawCircle: fn (vect.cpVect, types.cpFloat, types.cpFloat, cpSpaceDebugColor, cpSpaceDebugColor, ?*anyopaque) void,
    drawSegment: fn (vect.cpVect, vect.cpVect, cpSpaceDebugColor, ?*anyopaque) void,
    drawFatSegment: fn (vect.cpVect, vect.cpVect, types.cpFloat, cpSpaceDebugColor, cpSpaceDebugColor, ?*anyopaque) void,
    drawPolygon: fn (usize, []const vect.cpVect, types.cpFloat, cpSpaceDebugColor, cpSpaceDebugColor, ?*anyopaque) void,
    drawDot: fn (types.cpFloat, vect.cpVect, cpSpaceDebugColor, ?*anyopaque) void,
    colorForShape: fn (*shape_base.cpShape, ?*anyopaque) cpSpaceDebugColor,
    data: ?*anyopaque = null,
    flags: cpSpaceDebugFlags = .{},
    shapeOutlineColor: cpSpaceDebugColor = cpSpaceDebugColor.gray(),
    constraintColor: cpSpaceDebugColor = cpSpaceDebugColor.gray(),
    collisionPointColor: cpSpaceDebugColor = cpSpaceDebugColor.red(),
};

fn drawShape(shape: *shape_base.cpShape, options: cpSpaceDebugDrawOptions) void {
    const body = shape.body;
    const outline_color = options.shapeOutlineColor;
    const fill_color = options.colorForShape(shape, options.data);

    switch (shape.shape_type) {
        .circle => {
            const c = @as(*circle.cpCircleShape, @ptrCast(shape));
            const center = vect.cpvadd(body.p, vect.cpvrotate(c.offset, body.rotationVector()));
            options.drawCircle(center, body.a, c.radius, outline_color, fill_color, options.data);
        },
        .segment => {
            const s = @as(*segment.cpSegmentShape, @ptrCast(shape));
            const rot = body.rotationVector();
            const a = vect.cpvadd(body.p, vect.cpvrotate(s.a, rot));
            const b = vect.cpvadd(body.p, vect.cpvrotate(s.b, rot));
            options.drawFatSegment(a, b, s.radius, outline_color, fill_color, options.data);
        },
        .poly => {
            const p = @as(*poly.cpPolyShape, @ptrCast(shape));
            var transformed = std.ArrayList(vect.cpVect).init(std.heap.page_allocator);
            defer transformed.deinit();
            transformed.ensureTotalCapacity(p.vertices.len) catch {};
            const rot = body.rotationVector();
            for (p.vertices) |vertex| {
                transformed.append(vect.cpvadd(body.p, vect.cpvrotate(vertex, rot))) catch {};
            }
            options.drawPolygon(transformed.items.len, transformed.items, p.radius, outline_color, fill_color, options.data);
        },
    }
}

fn drawConstraint(entry: space_mod.ConstraintEntry, options: cpSpaceDebugDrawOptions) void {
    const a = entry.constraint.a.p;
    const b = entry.constraint.b.p;
    options.drawDot(3.0, a, options.constraintColor, options.data);
    options.drawDot(3.0, b, options.constraintColor, options.data);
    options.drawSegment(a, b, options.constraintColor, options.data);
}

pub fn cpSpaceDebugDraw(space: *space_mod.cpSpace, options: cpSpaceDebugDrawOptions) void {
    if (options.flags.draw_shapes) {
        for (space.shapes.items) |shape| {
            drawShape(shape, options);
        }
    }

    if (options.flags.draw_constraints) {
        for (space.constraints.items) |entry| {
            drawConstraint(entry, options);
        }
    }

    if (options.flags.draw_collision_points) {
        for (space.arbiters.items) |arb| {
            for (arb.contacts.constSlice()) |contact| {
                const start = contact.point;
                const end = vect.cpvadd(contact.point, vect.cpvmult(contact.normal, 2.0));
                options.drawSegment(start, end, options.collisionPointColor, options.data);
            }
        }
    }
}

test "space debug draw triggers callbacks" {
    var space = space_mod.cpSpace.init(std.testing.allocator);
    defer space.deinit();

    var body_a = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var body_b = shape_base.body_mod.cpBody.init(1.0, 1.0);
    var circle_shape = circle.cpCircleShape.init(&body_a, 0.5, vect.cpvzero);
    var constraint = constraint_base.cpConstraint.init(&body_a, &body_b);

    try space.addBody(&body_a);
    try space.addBody(&body_b);
    try space.addShape(&circle_shape.base);
    try space.addConstraint(&constraint, .{}, null);

    var counts = struct {
        circles: usize = 0,
        segments: usize = 0,
        dots: usize = 0,
    }{};

    const opts = cpSpaceDebugDrawOptions{
        .drawCircle = struct {
            fn call(
                _: vect.cpVect,
                _: types.cpFloat,
                _: types.cpFloat,
                _: cpSpaceDebugColor,
                _: cpSpaceDebugColor,
                data: ?*anyopaque,
            ) void {
                const state = @as(*@TypeOf(counts), @ptrCast(@alignCast(data.?)));
                state.circles += 1;
            }
        }.call,
        .drawSegment = struct {
            fn call(_: vect.cpVect, _: vect.cpVect, _: cpSpaceDebugColor, data: ?*anyopaque) void {
                const state = @as(*@TypeOf(counts), @ptrCast(@alignCast(data.?)));
                state.segments += 1;
            }
        }.call,
        .drawFatSegment = struct {
            fn call(
                _: vect.cpVect,
                _: vect.cpVect,
                _: types.cpFloat,
                _: cpSpaceDebugColor,
                _: cpSpaceDebugColor,
                _: ?*anyopaque,
            ) void {}
        }.call,
        .drawPolygon = struct {
            fn call(
                _: usize,
                _: []const vect.cpVect,
                _: types.cpFloat,
                _: cpSpaceDebugColor,
                _: cpSpaceDebugColor,
                _: ?*anyopaque,
            ) void {}
        }.call,
        .drawDot = struct {
            fn call(_: types.cpFloat, _: vect.cpVect, _: cpSpaceDebugColor, data: ?*anyopaque) void {
                const state = @as(*@TypeOf(counts), @ptrCast(@alignCast(data.?)));
                state.dots += 1;
            }
        }.call,
        .colorForShape = struct {
            fn call(_: *shape_base.cpShape, _: ?*anyopaque) cpSpaceDebugColor {
                return cpSpaceDebugColor.white();
            }
        }.call,
        .data = &counts,
    };

    cpSpaceDebugDraw(&space, opts);

    try std.testing.expect(counts.circles > 0);
    try std.testing.expect(counts.segments >= 0);
    try std.testing.expect(counts.dots > 0);
}
