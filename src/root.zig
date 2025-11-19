const std = @import("std");

pub const core = @import("./core.zig");
pub const space = @import("./space.zig");
pub const shape = @import("./shape.zig");
pub const constraint = @import("./constraint.zig");
pub const spatial_index = @import("./spatial_index.zig");
pub const collision = @import("./collision.zig");
pub const extras = @import("./extras.zig");
const types = core.types;
const vect = core.vect;
const bb = core.bb;
const transform = core.transform;

pub fn bufferedPrint() !void {
    var stdout_buffer: [1024]u8 = undefined;
    var stdout_writer = std.fs.File.stdout().writer(&stdout_buffer);
    const stdout = &stdout_writer.interface;

    const a = vect.cpv(3.0, 4.0);
    const b = vect.cpv(-1.0, 2.0);
    const sum = vect.cpvadd(a, b);
    const bounds = bb.cpBBNewForCircle(a, 1.0);

    try stdout.print(
        "cpvadd({d:.1},{d:.1}) + ({d:.1},{d:.1}) = ({d:.1},{d:.1}) | area={d:.1}\n",
        .{ a.x, a.y, b.x, b.y, sum.x, sum.y, bb.cpBBArea(bounds) },
    );

    const rotated = transform.cpTransformRotate(types.CP_PI / 2.0);
    const translated = transform.cpTransformTranslate(b);
    const composite = transform.cpTransformMult(translated, rotated);
    const transformed = transform.cpTransformPoint(composite, a);

    try stdout.print(
        "transform point -> ({d:.2},{d:.2})\n",
        .{ transformed.x, transformed.y },
    );

    try stdout.flush();
}

test "basic vector arithmetic" {
    const a = vect.cpv(1.0, 0.0);
    const b = vect.cpv(0.0, 1.0);
    try std.testing.expect(vect.cpvdot(a, b) == 0.0);
    try std.testing.expect(vect.cpvcross(a, b) == 1.0);
    try std.testing.expectApproxEqAbs(1.0, vect.cpvlength(a), 1e-9);
}
