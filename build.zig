const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const mod = b.addModule("chipmunk", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    const c_chipmunk = b.addStaticLibrary(.{
        .name = "chipmunk_c_reference",
        .target = target,
        .optimize = optimize,
    });
    c_chipmunk.addIncludePath(b.path("chipmunk2d/include"));
    c_chipmunk.addCSourceFiles(.{
        .files = &.{
            "chipmunk2d/src/chipmunk.c",
            "chipmunk2d/src/cpArbiter.c",
            "chipmunk2d/src/cpArray.c",
            "chipmunk2d/src/cpBBTree.c",
            "chipmunk2d/src/cpBody.c",
            "chipmunk2d/src/cpCollision.c",
            "chipmunk2d/src/cpConstraint.c",
            "chipmunk2d/src/cpDampedRotarySpring.c",
            "chipmunk2d/src/cpDampedSpring.c",
            "chipmunk2d/src/cpGearJoint.c",
            "chipmunk2d/src/cpGrooveJoint.c",
            "chipmunk2d/src/cpHashSet.c",
            "chipmunk2d/src/cpHastySpace.c",
            "chipmunk2d/src/cpMarch.c",
            "chipmunk2d/src/cpPinJoint.c",
            "chipmunk2d/src/cpPivotJoint.c",
            "chipmunk2d/src/cpPolyShape.c",
            "chipmunk2d/src/cpPolyline.c",
            "chipmunk2d/src/cpRatchetJoint.c",
            "chipmunk2d/src/cpRobust.c",
            "chipmunk2d/src/cpRotaryLimitJoint.c",
            "chipmunk2d/src/cpShape.c",
            "chipmunk2d/src/cpSimpleMotor.c",
            "chipmunk2d/src/cpSlideJoint.c",
            "chipmunk2d/src/cpSpace.c",
            "chipmunk2d/src/cpSpaceComponent.c",
            "chipmunk2d/src/cpSpaceDebug.c",
            "chipmunk2d/src/cpSpaceHash.c",
            "chipmunk2d/src/cpSpaceQuery.c",
            "chipmunk2d/src/cpSpaceStep.c",
            "chipmunk2d/src/cpSpatialIndex.c",
            "chipmunk2d/src/cpSweep1D.c",
        },
        .flags = &.{"-std=c99"},
    });
    c_chipmunk.linkLibC();

    const exe_root_module = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .imports = &.{.{ .name = "chipmunk", .module = mod }},
    });

    const exe = b.addExecutable(.{
        .name = "chipmunk",
        .root_module = exe_root_module,
    });

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the app");

    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const mod_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/root.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    mod_tests.linkLibrary(c_chipmunk);
    mod_tests.root_module.addIncludePath(b.path("chipmunk2d/include"));
    const run_mod_tests = b.addRunArtifact(mod_tests);

    const exe_tests = b.addTest(.{
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{.{ .name = "chipmunk", .module = mod }},
        }),
    });
    exe_tests.linkLibrary(c_chipmunk);
    exe_tests.root_module.addIncludePath(b.path("chipmunk2d/include"));
    const run_exe_tests = b.addRunArtifact(exe_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
}
