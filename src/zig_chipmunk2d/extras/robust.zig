const types = @import("../core/types.zig");
const vect = @import("../core/vect.zig");

pub fn cpCheckPointGreater(a: vect.cpVect, b: vect.cpVect, c: vect.cpVect) types.cpBool {
    return (b.y - a.y) * (a.x + b.x - 2.0 * c.x) > (b.x - a.x) * (a.y + b.y - 2.0 * c.y);
}

pub fn cpCheckAxis(v0: vect.cpVect, v1: vect.cpVect, p: vect.cpVect, n: vect.cpVect) types.cpBool {
    return vect.cpvdot(p, n) <= types.cpfmax(vect.cpvdot(v0, n), vect.cpvdot(v1, n));
}
