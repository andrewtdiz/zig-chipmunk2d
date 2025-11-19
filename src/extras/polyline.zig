const std = @import("std");
const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");

pub const cpPolyline = struct {
    allocator: std.mem.Allocator,
    verts: std.ArrayList(vect.cpVect),

    pub fn init(allocator: std.mem.Allocator, capacity: usize) cpPolyline {
        var verts = std.ArrayList(vect.cpVect).init(allocator);
        verts.ensureTotalCapacity(capacity) catch {};
        return .{ .allocator = allocator, .verts = verts };
    }

    pub fn deinit(self: *cpPolyline) void {
        self.verts.deinit();
    }

    pub fn count(self: cpPolyline) usize {
        return self.verts.items.len;
    }

    pub fn isClosed(self: cpPolyline) types.cpBool {
        if (self.count() <= 1) return false;
        return vect.cpveql(self.verts.items[0], self.verts.items[self.count() - 1]);
    }

    pub fn push(self: *cpPolyline, v: vect.cpVect) void {
        self.verts.append(v) catch {};
    }

    pub fn enqueue(self: *cpPolyline, v: vect.cpVect) void {
        self.verts.insert(0, v) catch {};
    }
};

fn sharpness(a: vect.cpVect, b: vect.cpVect, c: vect.cpVect) types.cpFloat {
    return vect.cpvdot(vect.cpvnormalize(vect.cpvsub(a, b)), vect.cpvnormalize(vect.cpvsub(c, b)));
}

pub fn cpPolylineSimplifyVertexes(line: *const cpPolyline, allocator: std.mem.Allocator, tol: types.cpFloat) cpPolyline {
    if (line.count() <= 2) {
        var copy = cpPolyline.init(allocator, line.count());
        copy.verts.appendSlice(line.verts.items) catch {};
        return copy;
    }

    var reduced = cpPolyline.init(allocator, 2);
    reduced.push(line.verts.items[0]);
    reduced.push(line.verts.items[1]);

    const min_sharp = -types.cpfcos(tol);
    var i: usize = 2;
    while (i < line.count()) : (i += 1) {
        const vert = line.verts.items[i];
        const sharp = sharpness(reduced.verts.items[reduced.count() - 2], reduced.verts.items[reduced.count() - 1], vert);
        if (sharp <= min_sharp) {
            reduced.verts.items[reduced.count() - 1] = vert;
        } else {
            reduced.push(vert);
        }
    }

    if (line.isClosed() and sharpness(
        reduced.verts.items[reduced.count() - 2],
        reduced.verts.items[0],
        reduced.verts.items[1],
    ) < min_sharp) {
        reduced.verts.items[0] = reduced.verts.items[reduced.count() - 2];
        _ = reduced.verts.pop();
    }

    return reduced;
}

fn polylineIsShort(points: []const vect.cpVect, start: usize, end: usize, min: types.cpFloat) types.cpBool {
    var length: types.cpFloat = 0.0;
    var i = start;
    while (true) {
        const next = (i + 1) % points.len;
        length += vect.cpvdist(points[i], points[next]);
        if (length > min) return false;
        if (next == end) break;
        i = next;
    }
    return true;
}

fn douglasPeucker(
    verts: []const vect.cpVect,
    reduced: *cpPolyline,
    length: usize,
    start: usize,
    end: usize,
    min: types.cpFloat,
    tol: types.cpFloat,
) void {
    const segment_len = (end + length - start) % length;
    if (segment_len < 2) return;

    const a = verts[start];
    const b = verts[end];
    if (vect.cpvnear(a, b, min) and polylineIsShort(verts, start, end, min)) return;

    var max_val: types.cpFloat = 0.0;
    var max_i = start;

    const n = vect.cpvnormalize(vect.cpvperp(vect.cpvsub(b, a)));
    const d = vect.cpvdot(n, a);

    var idx = (start + 1) % length;
    while (idx != end) : (idx = (idx + 1) % length) {
        const dist = @abs(vect.cpvdot(n, verts[idx]) - d);
        if (dist > max_val) {
            max_val = dist;
            max_i = idx;
        }
    }

    if (max_val > tol) {
        douglasPeucker(verts, reduced, length, start, max_i, min, tol);
        reduced.push(verts[max_i]);
        douglasPeucker(verts, reduced, length, max_i, end, min, tol);
    }
}

pub fn cpPolylineSimplifyCurves(line: *const cpPolyline, allocator: std.mem.Allocator, tol: types.cpFloat) cpPolyline {
    var reduced = cpPolyline.init(allocator, line.count());
    const min = tol / 2.0;

    if (line.isClosed()) {
        const start: usize = 0;
        const end: usize = line.count() - 2;
        reduced.push(line.verts.items[start]);
        douglasPeucker(line.verts.items, &reduced, line.count() - 1, start, end, min, tol);
        reduced.push(line.verts.items[end]);
        douglasPeucker(line.verts.items, &reduced, line.count() - 1, end, start, min, tol);
        reduced.push(line.verts.items[start]);
    } else if (line.count() > 0) {
        reduced.push(line.verts.items[0]);
        douglasPeucker(line.verts.items, &reduced, line.count(), 0, line.count() - 1, min, tol);
        reduced.push(line.verts.items[line.count() - 1]);
    }

    return reduced;
}

pub const cpPolylineSet = struct {
    allocator: std.mem.Allocator,
    lines: std.ArrayList(cpPolyline),

    pub fn init(allocator: std.mem.Allocator) cpPolylineSet {
        return .{ .allocator = allocator, .lines = std.ArrayList(cpPolyline).init(allocator) };
    }

    pub fn deinit(self: *cpPolylineSet) void {
        for (self.lines.items) |*line| line.deinit();
        self.lines.deinit();
    }

    fn findEnds(self: cpPolylineSet, v: vect.cpVect) ?usize {
        for (self.lines.items, 0..) |line, idx| {
            if (line.count() > 0 and vect.cpveql(line.verts.items[line.count() - 1], v)) return idx;
        }
        return null;
    }

    fn findStarts(self: cpPolylineSet, v: vect.cpVect) ?usize {
        for (self.lines.items, 0..) |line, idx| {
            if (line.count() > 0 and vect.cpveql(line.verts.items[0], v)) return idx;
        }
        return null;
    }

    fn pushLine(self: *cpPolylineSet, line: cpPolyline) void {
        self.lines.append(line) catch {};
    }

    fn addLine(self: *cpPolylineSet, v0: vect.cpVect, v1: vect.cpVect) void {
        var line = cpPolyline.init(self.allocator, 2);
        line.push(v0);
        line.push(v1);
        self.pushLine(line);
    }

    fn join(self: *cpPolylineSet, before: usize, after: usize) void {
        var lbefore = &self.lines.items[before];
        var lafter = self.lines.items[after];
        lbefore.verts.appendSlice(lafter.verts.items) catch {};
        lafter.deinit();
        _ = self.lines.orderedRemove(after);
    }

    pub fn collectSegment(self: *cpPolylineSet, v0: vect.cpVect, v1: vect.cpVect) void {
        const before = self.findEnds(v0);
        const after = self.findStarts(v1);

        if (before) |b| {
            if (after) |a| {
                if (b == a) {
                    self.lines.items[b].push(v1);
                } else {
                    self.join(b, a);
                }
            } else {
                self.lines.items[b].push(v1);
            }
        } else if (after) |a| {
            self.lines.items[a].enqueue(v0);
        } else {
            self.addLine(v0, v1);
        }
    }
};

pub fn cpPolylineIsClosed(line: *cpPolyline) types.cpBool {
    return line.isClosed();
}

test "polyline simplification reduces nearly collinear vertices" {
    var line = cpPolyline.init(std.testing.allocator, 4);
    defer line.deinit();
    line.push(vect.cpv(0.0, 0.0));
    line.push(vect.cpv(1.0, 0.0));
    line.push(vect.cpv(2.0, 0.01));
    line.push(vect.cpv(3.0, 0.0));

    var simplified = cpPolylineSimplifyVertexes(&line, std.testing.allocator, 0.1);
    defer simplified.deinit();

    try std.testing.expect(simplified.count() < line.count());
}
