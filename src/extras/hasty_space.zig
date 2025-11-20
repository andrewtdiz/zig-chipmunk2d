const std = @import("std");
const space_mod = @import("../space/space.zig");
const body_mod = @import("../space/body.zig");
const shape_base = @import("../shape/shape_base.zig");
const constraint_base = @import("../constraint/constraint_base.zig");
const types = @import("../core/types.zig");

const JobPayload = union(enum) {
    constraint: ConstraintJob,
    arbiter: ArbiterJob,
    broad_phase: BroadPhaseJob,
};

const ConstraintJob = struct {
    items: []space_mod.ConstraintEntry,
    phase: space_mod.ConstraintCallbackPhase,
    dt: types.cpFloat,
    dt_coef: types.cpFloat,
};

const ArbiterJob = struct {
    space: *space_mod.cpSpace,
};

const BroadPhaseJob = struct {
    space: *space_mod.cpSpace,
    mutex: *std.Thread.Mutex,
};

const Job = struct {
    payload: JobPayload,
    start: usize,
    end: usize,
};

const WorkerContext = struct {
    queue: *JobQueue,
};

const Worker = struct {
    thread: std.Thread,

    fn join(self: *Worker) void {
        self.thread.join();
    }
};

const JobQueue = struct {
    allocator: std.mem.Allocator,
    jobs: std.ArrayList(Job),
    mutex: std.Thread.Mutex = .{},
    work_ready: std.Thread.Condition = .{},
    drained: std.Thread.Condition = .{},
    pending: usize = 0,
    shutdown: bool = false,

    fn init(allocator: std.mem.Allocator) JobQueue {
        return .{ .allocator = allocator, .jobs = std.ArrayList(Job).init(allocator) };
    }

    fn deinit(self: *JobQueue) void {
        self.jobs.deinit();
    }

    fn push(self: *JobQueue, job: Job) !void {
        self.mutex.lock();
        defer self.mutex.unlock();
        try self.jobs.append(job);
        self.pending += 1;
        self.work_ready.signal();
    }

    fn take(self: *JobQueue) ?Job {
        self.mutex.lock();
        defer self.mutex.unlock();
        while (self.jobs.items.len == 0 and !self.shutdown) {
            self.work_ready.wait(&self.mutex);
        }
        if (self.jobs.items.len == 0) {
            return null;
        }
        return self.jobs.pop();
    }

    fn complete(self: *JobQueue) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        if (self.pending == 0) return;
        self.pending -= 1;
        if (self.pending == 0) {
            self.drained.broadcast();
        }
    }

    fn waitAll(self: *JobQueue) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        while (self.pending > 0) {
            self.drained.wait(&self.mutex);
        }
    }

    fn requestStop(self: *JobQueue) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        self.shutdown = true;
        self.work_ready.broadcast();
    }

    fn resetAfterStop(self: *JobQueue) void {
        self.mutex.lock();
        defer self.mutex.unlock();
        self.jobs.clearRetainingCapacity();
        self.pending = 0;
        self.shutdown = false;
    }
};

fn workerMain(ctx: *WorkerContext) void {
    while (true) {
        const job = ctx.queue.take() orelse return;
        executeJob(job);
        ctx.queue.complete();
    }
}

fn executeJob(job: Job) void {
    switch (job.payload) {
        .constraint => |payload| {
            space_mod.runConstraintCallbackRange(
                payload.items,
                payload.dt,
                payload.dt_coef,
                payload.phase,
                job.start,
                job.end,
            );
        },
        .arbiter => |payload| {
            space_mod.resolveArbitersRange(payload.space, job.start, job.end);
        },
        .broad_phase => |payload| {
            space_mod.resolveCollisionsRange(payload.space, job.start, job.end, payload.mutex);
        },
    }
}

pub const cpHastySpace = struct {
    space: space_mod.cpSpace,
    queue: JobQueue,
    workers: []Worker = &.{},
    contexts: []WorkerContext = &.{},
    thread_target: usize = 1,
    parallel_threshold: usize = 32,
    broadphase_mutex: std.Thread.Mutex = .{},

    pub fn init(allocator: std.mem.Allocator) cpHastySpace {
        return .{
            .space = space_mod.cpSpace.init(allocator),
            .queue = JobQueue.init(allocator),
        };
    }

    pub fn deinit(self: *cpHastySpace) void {
        self.resizeWorkers(1) catch {};
        self.queue.deinit();
        self.space.deinit();
    }

    pub fn setThreads(self: *cpHastySpace, count: usize) !void {
        try self.resizeWorkers(count);
    }

    pub fn threads(self: *const cpHastySpace) usize {
        return self.thread_target;
    }

    pub fn addBody(self: *cpHastySpace, body: *body_mod.cpBody) !void {
        try self.space.addBody(body);
    }

    pub fn addShape(self: *cpHastySpace, shape: *shape_base.cpShape) !void {
        try self.space.addShape(shape);
    }

    pub fn addConstraint(
        self: *cpHastySpace,
        constraint: *constraint_base.cpConstraint,
        ops: space_mod.ConstraintOps,
        payload: ?*anyopaque,
    ) !void {
        try self.space.addConstraint(constraint, ops, payload);
    }

    pub fn step(self: *cpHastySpace, dt: types.cpFloat) void {
        const dt_coef: types.cpFloat = if (dt != 0.0) dt else 1.0;
        space_mod.startBroadPhase(&self.space);
        space_mod.updateVelocities(&self.space, dt);
        space_mod.runConstraintCallback(self.space.constraints.items, dt, dt_coef, .preStep);
        space_mod.runConstraintCallback(self.space.constraints.items, dt, dt_coef, .applyCachedImpulse);
        space_mod.updateShapeCaches(&self.space);

        const dynamic_len = self.space.dynamic_shapes.items.len;
        if (self.workers.len == 0 or dynamic_len < self.parallel_threshold) {
            space_mod.resolveCollisionsRange(&self.space, 0, dynamic_len, null);
        } else {
            const payload = JobPayload{ .broad_phase = .{ .space = &self.space, .mutex = &self.broadphase_mutex } };
            self.dispatchJobs(payload, dynamic_len);
        }

        var iteration: usize = 0;
        while (iteration < self.space.iterations) : (iteration += 1) {
            self.runConstraintPhase(.applyImpulse, dt, dt_coef);
            self.runArbiterPhase();
        }

        space_mod.postSolveArbiters(&self.space);
        space_mod.integratePositions(&self.space, dt);
        space_mod.runConstraintCallback(self.space.constraints.items, dt, dt_coef, .postStep);
        space_mod.runPostSteps(&self.space);
        space_mod.finishBroadPhase(&self.space);
    }

    fn resizeWorkers(self: *cpHastySpace, requested: usize) !void {
        const clamped = if (requested == 0) 1 else requested;
        if (clamped == self.thread_target and self.workers.len == if (clamped > 0) clamped - 1 else 0) return;

        self.queue.waitAll();
        if (self.workers.len > 0) {
            self.queue.requestStop();
            for (self.workers) |*worker| {
                worker.join();
            }
            self.queue.resetAfterStop();
            self.space.allocator.free(self.workers);
            self.space.allocator.free(self.contexts);
            self.workers = &.{};
            self.contexts = &.{};
        }

        self.thread_target = clamped;
        const worker_count = if (clamped > 0) clamped - 1 else 0;
        if (worker_count == 0) return;

        self.contexts = try self.space.allocator.alloc(WorkerContext, worker_count);
        self.workers = try self.space.allocator.alloc(Worker, worker_count);
        for (self.contexts, 0..) |*ctx, i| {
            ctx.* = .{ .queue = &self.queue };
            self.workers[i] = .{ .thread = try std.Thread.spawn(.{}, workerMain, .{ctx}) };
        }
    }

    fn runConstraintPhase(self: *cpHastySpace, phase: space_mod.ConstraintCallbackPhase, dt: types.cpFloat, dt_coef: types.cpFloat) void {
        const constraints = self.space.constraints.items;
        if (constraints.len == 0) return;
        if (phase != .applyImpulse or self.workers.len == 0 or constraints.len < self.parallel_threshold) {
            space_mod.runConstraintCallback(constraints, dt, dt_coef, phase);
            return;
        }

        const payload = JobPayload{ .constraint = .{
            .items = constraints,
            .phase = phase,
            .dt = dt,
            .dt_coef = dt_coef,
        } };
        self.dispatchJobs(payload, constraints.len);
    }

    fn runArbiterPhase(self: *cpHastySpace) void {
        const arb_len = self.space.arbiters.items.len;
        if (arb_len == 0) return;
        if (self.workers.len == 0 or arb_len < self.parallel_threshold) {
            space_mod.resolveArbiters(&self.space);
            return;
        }

        const payload = JobPayload{ .arbiter = .{ .space = &self.space } };
        self.dispatchJobs(payload, arb_len);
    }

    fn dispatchJobs(self: *cpHastySpace, payload: JobPayload, total: usize) void {
        if (total == 0) return;
        const worker_shares = self.workers.len + 1;
        var chunk = total / worker_shares;
        if (chunk == 0) chunk = 1;
        const inline_end = @min(chunk, total);

        var start: usize = inline_end;
        while (start < total) : (start = @min(start + chunk, total)) {
            const end = @min(start + chunk, total);
            self.queue.push(.{ .payload = payload, .start = start, .end = end }) catch {
                executeJob(.{ .payload = payload, .start = start, .end = end });
                continue;
            };
        }

        executeJob(.{ .payload = payload, .start = 0, .end = inline_end });
        self.queue.waitAll();
    }
};
