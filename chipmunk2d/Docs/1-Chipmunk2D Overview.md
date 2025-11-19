# Chipmunk2D Overview

<details>
<summary>Relevant source files</summary>

The following files were used as context for generating this wiki page:

- [LICENSE.txt](LICENSE.txt)
- [README.textile](README.textile)
- [TODO.txt](TODO.txt)
- [VERSION.txt](VERSION.txt)
- [doc-src/chipmunk-docs.textile](doc-src/chipmunk-docs.textile)
- [include/chipmunk/chipmunk.h](include/chipmunk/chipmunk.h)
- [src/CMakeLists.txt](src/CMakeLists.txt)
- [src/chipmunk.c](src/chipmunk.c)

</details>



This document provides an overview of the Chipmunk2D physics engine, introducing its core purpose, key features, and overall system architecture. It serves as the entry point for understanding how the various components of the physics engine work together to provide 2D rigid body simulation.

For detailed information about specific subsystems, see [Core Physics Engine](#2), [API and Integration](#3), and [Advanced Features](#4).

## What is Chipmunk2D?

Chipmunk2D is a simple, lightweight, fast and portable 2D rigid body physics library written in C. It provides real-time physics simulation capabilities specifically designed for 2D video games and interactive applications. The library is distributed under the MIT license, making it freely available for both commercial and non-commercial use.

The primary goal of Chipmunk2D is to give 2D developers access to the same quality of physics simulation found in modern 3D games, while maintaining simplicity and performance optimized for 2D scenarios.

**Sources:** [README.textile:9-11](), [doc-src/chipmunk-docs.textile:5-9]()

## Key Features and Capabilities

Chipmunk2D provides a comprehensive set of physics simulation features:

| Feature Category | Capabilities |
|------------------|-------------|
| **Collision Primitives** | Circle, convex polygon, and beveled line segment shapes |
| **Multi-Shape Bodies** | Multiple collision primitives can be attached to a single rigid body |
| **Collision Detection** | Fast broad-phase using bounding box trees or spatial hashing |
| **Contact Solving** | Extremely fast impulse solving using Erin Catto's contact persistence algorithm |
| **Performance Optimization** | Sleeping objects, spatial indexing, and optional multithreading |
| **Collision Callbacks** | Event-based collision handling with user-definable object types |
| **Collision Filtering** | Flexible system with layers, exclusion groups, and callbacks |
| **Spatial Queries** | Point, segment (raycasting), shape, and bounding box queries |
| **Joints and Constraints** | Large variety of joints for vehicles, ragdolls, and mechanical systems |
| **Cross-Platform** | Lightweight C99 implementation with no external dependencies |

**Sources:** [README.textile:13-33](), [doc-src/chipmunk-docs.textile:115-121]()

## Core Architecture Overview

Chipmunk2D follows a modular architecture with distinct layers for core physics simulation, language bindings, and applications:

```mermaid
graph TB
    subgraph "Application Layer"
        Games["Game Applications"]
        Demos["Demo Applications"]
        Tools["Development Tools"]
    end
    
    subgraph "Language Bindings Layer" 
        ObjC["ObjectiveChipmunk<br/>Objective-C Wrapper"]
        FFI["chipmunk_ffi<br/>Foreign Function Interface"]
        Bindings["Other Language<br/>Bindings"]
    end
    
    subgraph "Core Physics Engine (C Library)"
        PublicAPI["Public C API<br/>chipmunk.h"]
        Space["cpSpace<br/>Physics World"]
        Bodies["cpBody<br/>Rigid Bodies"]
        Shapes["cpShape<br/>Collision Geometry"]
        Constraints["cpConstraint<br/>Joints & Springs"]
        Collision["Collision Detection<br/>GJK/EPA Algorithms"]
        SpatialIndex["cpSpatialIndex<br/>Broad-Phase Optimization"]
        CoreTypes["Core Types<br/>cpVect, cpBB, cpTransform"]
    end
    
    subgraph "Build and Platform Layer"
        CMake["CMake Build System"]
        Xcode["Xcode Projects"]
        MSVC["Visual Studio Projects"]
    end
    
    Games --> ObjC
    Games --> PublicAPI
    Demos --> PublicAPI
    Tools --> FFI
    
    ObjC --> PublicAPI
    FFI --> PublicAPI
    Bindings --> PublicAPI
    
    PublicAPI --> Space
    PublicAPI --> Bodies
    PublicAPI --> Shapes
    PublicAPI --> Constraints
    
    Space --> Collision
    Space --> SpatialIndex
    Bodies --> CoreTypes
    Shapes --> CoreTypes
    Collision --> CoreTypes
    
    CMake --> PublicAPI
    Xcode --> PublicAPI
    Xcode --> ObjC
    MSVC --> PublicAPI
```

**Sources:** [include/chipmunk/chipmunk.h:44-127](), [src/CMakeLists.txt:1-60]()

## Core Object Types

The physics simulation revolves around four fundamental object types that map directly to C structures in the codebase:

```mermaid
graph LR
    subgraph "Physics World"
        cpSpace["cpSpace<br/>Physics Container"]
    end
    
    subgraph "Physical Objects"
        cpBody["cpBody<br/>Mass Properties<br/>• Position/Velocity<br/>• Mass/Moment<br/>• Forces/Torques"]
        
        cpShape["cpShape<br/>Collision Geometry<br/>• cpCircleShape<br/>• cpPolyShape<br/>• cpSegmentShape"]
        
        cpConstraint["cpConstraint<br/>Mechanical Connections<br/>• cpPinJoint<br/>• cpDampedSpring<br/>• cpSimpleMotorJoint"]
    end
    
    subgraph "Collision Processing"
        cpArbiter["cpArbiter<br/>Collision Information<br/>• Contact Points<br/>• Collision Normal<br/>• Impulse Data"]
    end
    
    subgraph "Supporting Types"
        cpVect["cpVect<br/>2D Vector<br/>• x, y coordinates<br/>• Vector operations"]
        
        cpBB["cpBB<br/>Bounding Box<br/>• left, bottom<br/>• right, top"]
        
        cpTransform["cpTransform<br/>2x3 Affine Transform<br/>• Position<br/>• Rotation<br/>• Scale"]
    end
    
    cpSpace --> cpBody
    cpSpace --> cpShape
    cpSpace --> cpConstraint
    
    cpBody --> cpShape
    cpBody --> cpConstraint
    
    cpShape --> cpArbiter
    cpConstraint --> cpArbiter
    
    cpBody --> cpVect
    cpBody --> cpTransform
    cpShape --> cpBB
    cpShape --> cpVect
```

**Sources:** [include/chipmunk/chipmunk.h:85-111](), [doc-src/chipmunk-docs.textile:115-122]()

## Physics Simulation Process

The core simulation operates through a step-based process managed by `cpSpace`:

| Phase | Function | Purpose |
|-------|----------|---------|
| **Integration** | Position/Velocity Updates | Apply forces and update object positions |
| **Broad Phase** | `cpSpatialIndex` | Identify potentially colliding object pairs |
| **Narrow Phase** | `cpCollide` functions | Calculate exact collision information |
| **Constraint Solving** | Impulse Solver | Resolve collisions and joint constraints |
| **Callbacks** | User-defined handlers | Handle collision events and custom logic |

**Sources:** [doc-src/chipmunk-docs.textile:113-122]()

## Version and Compatibility

Chipmunk2D is currently at version 7.0.3, representing a mature and stable API. Key version information is defined in the main header:

- **Major Version:** 7 - Significant API changes and new features
- **Minor Version:** 0 - Feature additions within API compatibility
- **Patch Version:** 3 - Bug fixes and minor improvements

The library maintains backward compatibility within major versions and provides migration guidance for major version upgrades.

**Sources:** [include/chipmunk/chipmunk.h:128-131](), [src/chipmunk.c:59](), [src/CMakeLists.txt:7-11]()

## Language Bindings and Integration

While the core engine is written in C, Chipmunk2D supports multiple programming languages:

| Binding Type | Implementation | Target Languages |
|--------------|----------------|------------------|
| **Native C API** | `chipmunk.h` | C, C++, Objective-C |
| **Objective-C Wrapper** | ObjectiveChipmunk | iOS/macOS applications |
| **Foreign Function Interface** | `chipmunk_ffi.h` | Python, Ruby, Lua, etc. |
| **Community Bindings** | Third-party | Java, C#, JavaScript, etc. |

The C API is designed to be easily wrapped by higher-level languages while maintaining full access to all physics engine capabilities.

**Sources:** [include/chipmunk/chipmunk.h:44-46](), [README.textile:31](), [doc-src/chipmunk-docs.textile:14-16]()

## Build System and Platform Support

Chipmunk2D supports multiple build systems and platforms:

```mermaid
graph TB
    subgraph "Build Systems"
        CMake["CMake<br/>Cross-platform build<br/>• Unix/Linux<br/>• Windows<br/>• macOS"]
        
        Xcode["Xcode Projects<br/>Apple platforms<br/>• macOS<br/>• iOS<br/>• Static libraries"]
        
        MSVC["Visual Studio<br/>Windows development<br/>• MSVC 10<br/>• MSVC 9"]
    end
    
    subgraph "Target Platforms"
        Desktop["Desktop<br/>• Windows<br/>• macOS<br/>• Linux"]
        
        Mobile["Mobile<br/>• iOS<br/>• Android"]
        
        Embedded["Embedded<br/>• ARM processors<br/>• NEON optimizations"]
    end
    
    CMake --> Desktop
    CMake --> Mobile
    Xcode --> Mobile
    MSVC --> Desktop
    
    Desktop --> Embedded
    Mobile --> Embedded
```

**Sources:** [src/CMakeLists.txt:1-60](), [README.textile:39-48]()15:T1f46

