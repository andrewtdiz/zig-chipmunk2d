# Collision Detection System

<details>
<summary>Relevant source files</summary>

The following files were used as context for generating this wiki page:

- [src/cpCollision.c](src/cpCollision.c)

</details>



This document covers the collision detection system in Chipmunk2D, which determines when and how shapes collide. The system implements the GJK (Gilbert-Johnson-Keerthi) algorithm for distance calculation and the EPA (Expanding Polytope Algorithm) for penetration depth calculation, along with optimized functions for primitive shape collisions.

For information about the shapes themselves and their properties, see [Collision Shapes](#2.4). For spatial indexing and broad-phase collision detection, see [Spatial Indexing and Optimization](#2.5).

## System Overview

The collision detection system serves as the narrow-phase collision detection component of the physics engine. When the broad-phase system identifies potentially colliding shape pairs, this system performs precise geometric calculations to determine actual collisions, contact points, and separation distances.

```mermaid
flowchart TD
    SpaceCollide["cpSpaceCollideShapes()"] --> CollideEntry["cpCollide()"]
    
    CollideEntry --> ShapeSort["Sort shapes by type"]
    ShapeSort --> DispatchTable["BuiltinCollisionFuncs dispatch"]
    
    DispatchTable --> CircleCircle["CircleToCircle()"]
    DispatchTable --> CircleSegment["CircleToSegment()"]
    DispatchTable --> SegmentSegment["SegmentToSegment()"]
    DispatchTable --> CirclePoly["CircleToPoly()"]
    DispatchTable --> SegmentPoly["SegmentToPoly()"]
    DispatchTable --> PolyPoly["PolyToPoly()"]
    
    CircleCircle --> ContactInfo["cpCollisionInfo"]
    CircleSegment --> ContactInfo
    SegmentSegment --> GJK["GJK Algorithm"]
    CirclePoly --> GJK
    SegmentPoly --> GJK
    PolyPoly --> GJK
    
    GJK --> EPA["EPA Algorithm (if overlapping)"]
    GJK --> ClosestPoints["ClosestPointsNew()"]
    EPA --> ClosestPoints
    
    ClosestPoints --> ContactClipping["ContactPoints()"]
    ContactClipping --> ContactInfo
    
    ContactInfo --> SpaceArbiter["cpArbiter (via cpSpace)"]
```

**Sources:** [src/cpCollision.c:701-726]()

## Algorithm Architecture

The collision detection system uses different algorithms depending on shape complexity and collision requirements. Simple shapes use direct geometric calculations, while complex shapes use the GJK/EPA algorithm pair.

### GJK Algorithm Implementation

The GJK algorithm calculates the minimum distance between two convex shapes by iteratively sampling their Minkowski difference. The implementation uses recursive function calls with early termination conditions.

```mermaid
flowchart TD
    GJKStart["GJK()"] --> InitialPoints["Get initial MinkowskiPoints"]
    InitialPoints --> CachedCheck{"Use cached collision ID?"}
    
    CachedCheck -->|Yes| CachedPoints["MinkowskiPointNew from cached indices"]
    CachedCheck -->|No| BBoxGuess["Support points from bbox centers"]
    
    CachedPoints --> GJKRecurse
    BBoxGuess --> GJKRecurse["GJKRecurse()"]
    
    GJKRecurse --> OriginCheck{"Origin behind axis?"}
    OriginCheck -->|Yes| FlipAndRecurse["Flip v0, v1 and recurse"]
    OriginCheck -->|No| FindSupport["Support() in perpendicular direction"]
    
    FindSupport --> TriangleCheck{"Triangle contains origin?"}
    TriangleCheck -->|Yes| EPACall["EPA() for penetration depth"]
    TriangleCheck -->|No| AxisCheck{"Existing edge closest?"}
    
    AxisCheck -->|Yes| ReturnClosest["ClosestPointsNew()"]
    AxisCheck -->|No| ChooseVertex["Choose v0 or v1 to drop"]
    
    ChooseVertex --> RecurseAgain["GJKRecurse() with new pair"]
    FlipAndRecurse --> GJKRecurse
    RecurseAgain --> GJKRecurse
    
    EPACall --> ReturnEPA["EPA result"]
    ReturnClosest --> FinalResult["ClosestPoints"]
    ReturnEPA --> FinalResult
```

**Sources:** [src/cpCollision.c:415-472](), [src/cpCollision.c:347-392]()

### EPA Algorithm for Penetration Depth

When GJK determines that shapes are overlapping, EPA calculates the exact penetration depth and separation vector by expanding the convex hull of Minkowski difference points.

```mermaid
flowchart TD
    EPAStart["EPA()"] --> InitHull["Initialize 3-point hull"]
    InitHull --> EPARecurse["EPARecurse()"]
    
    EPARecurse --> FindClosest["Find closest hull edge to origin"]
    FindClosest --> SupportQuery["Support() perpendicular to edge"]
    
    SupportQuery --> DuplicateCheck{"Support point duplicate?"}
    DuplicateCheck -->|Yes| FinalEdge["Return ClosestPointsNew()"]
    DuplicateCheck -->|No| HullExpand["Expand convex hull"]
    
    HullExpand --> RebuildHull["Rebuild hull with new point"]
    RebuildHull --> RecurseAgain["EPARecurse() with new hull"]
    
    RecurseAgain --> EPARecurse
    FinalEdge --> EPAResult["ClosestPoints with penetration"]
```

**Sources:** [src/cpCollision.c:334-342](), [src/cpCollision.c:268-332]()

## Shape-Specific Collision Functions

The system provides optimized collision functions for primitive shape pairs that avoid the computational overhead of GJK/EPA when direct geometric calculations are simpler and faster.

| Shape Pair | Function | Algorithm | Complexity |
|------------|----------|-----------|------------|
| Circle-Circle | `CircleToCircle` | Distance comparison | O(1) |
| Circle-Segment | `CircleToSegment` | Point-to-line distance | O(1) |
| Segment-Segment | `SegmentToSegment` | GJK with tangent rejection | O(log n) |
| Circle-Poly | `CircleToPoly` | GJK | O(log n) |
| Segment-Poly | `SegmentToPoly` | GJK with tangent rejection | O(log n) |
| Poly-Poly | `PolyToPoly` | GJK | O(log n) |

```mermaid
flowchart LR
    ShapeTypes["Shape Type Pairs"] --> Dispatch["BuiltinCollisionFuncs[a->type + b->type*CP_NUM_SHAPES]"]
    
    Dispatch --> Primitive["Primitive Collisions"]
    Dispatch --> Complex["Complex Collisions"]
    
    Primitive --> CircleCircle["CircleToCircle"]
    Primitive --> CircleSegment["CircleToSegment"]
    
    Complex --> SegmentSegment["SegmentToSegment"]
    Complex --> CirclePoly["CircleToPoly"]
    Complex --> SegmentPoly["SegmentToPoly"]
    Complex --> PolyPoly["PolyToPoly"]
    
    CircleCircle --> DirectCalc["Direct geometric calculation"]
    CircleSegment --> DirectCalc
    
    SegmentSegment --> GJKFlow["GJK + tangent validation"]
    CirclePoly --> GJKFlow
    SegmentPoly --> GJKFlow
    PolyPoly --> GJKFlow
```

**Sources:** [src/cpCollision.c:688-698](), [src/cpCollision.c:524-679]()

## Support Point System

The support point system provides the foundation for GJK/EPA algorithms by calculating the furthest point on a shape in any given direction. Each shape type implements its own support point function optimized for its geometric properties.

```mermaid
flowchart TD
    SupportContext["SupportContext"] --> ShapeTypes["Shape Type Dispatch"]
    
    ShapeTypes --> CircleSupport["CircleSupportPoint()"]
    ShapeTypes --> SegmentSupport["SegmentSupportPoint()"]
    ShapeTypes --> PolySupport["PolySupportPoint()"]
    
    CircleSupport --> CircleResult["SupportPoint{center, 0}"]
    SegmentSupport --> SegmentChoice{"cpvdot(ta,n) > cpvdot(tb,n)?"}
    PolySupport --> PolyIndex["PolySupportPointIndex()"]
    
    SegmentChoice -->|Yes| SegmentA["SupportPoint{ta, 0}"]
    SegmentChoice -->|No| SegmentB["SupportPoint{tb, 1}"]
    
    PolyIndex --> PolyResult["SupportPoint{planes[i].v0, i}"]
    
    CircleResult --> MinkowskiPoint["MinkowskiPointNew()"]
    SegmentA --> MinkowskiPoint
    SegmentB --> MinkowskiPoint
    PolyResult --> MinkowskiPoint
    
    MinkowskiPoint --> GJKAlgorithm["Used by GJK/EPA"]
```

**Sources:** [src/cpCollision.c:93-117](), [src/cpCollision.c:130-148]()

## Contact Generation and Clipping

When shapes are determined to be colliding, the system generates contact points that represent the specific locations and properties of the collision. This involves clipping support edges against each other to find intersection points.

```mermaid
flowchart TD
    ClosestPoints["ClosestPoints from GJK/EPA"] --> ContactPoints["ContactPoints()"]
    
    ContactPoints --> DistanceCheck{"points.d <= e1.r + e2.r?"}
    DistanceCheck -->|No| NoContact["No collision"]
    DistanceCheck -->|Yes| GetEdges["Get support edges"]
    
    GetEdges --> Edge1["SupportEdgeForPoly/Segment(shape1, n)"]
    GetEdges --> Edge2["SupportEdgeForPoly/Segment(shape2, -n)"]
    
    Edge1 --> EdgeClip["Edge clipping algorithm"]
    Edge2 --> EdgeClip
    
    EdgeClip --> ProjectPoints["Project edge endpoints"]
    ProjectPoints --> ClampAndTest["Clamp to edges and test overlap"]
    
    ClampAndTest --> Contact1{"Contact point 1 valid?"}
    ClampAndTest --> Contact2{"Contact point 2 valid?"}
    
    Contact1 -->|Yes| PushContact1["cpCollisionInfoPushContact()"]
    Contact2 -->|Yes| PushContact2["cpCollisionInfoPushContact()"]
    
    PushContact1 --> FinalInfo["cpCollisionInfo"]
    PushContact2 --> FinalInfo
    NoContact --> FinalInfo
```

**Sources:** [src/cpCollision.c:476-518](), [src/cpCollision.c:163-195](), [src/cpCollision.c:44-55]()

## Integration with Physics Engine

The collision detection system integrates with the broader physics simulation through the `cpCollisionInfo` structure, which packages collision results for use by the constraint solver and contact management systems.

```mermaid
flowchart LR
    cpSpace["cpSpace"] --> SpaceCollide["cpSpaceCollideShapes()"]
    SpaceCollide --> cpCollide["cpCollide()"]
    
    cpCollide --> CollisionInfo["cpCollisionInfo"]
    CollisionInfo --> ContactArray["cpContact array"]
    CollisionInfo --> Normal["collision normal"]
    CollisionInfo --> CollisionID["collision ID for caching"]
    
    ContactArray --> cpArbiter["cpArbiter creation/update"]
    Normal --> cpArbiter
    CollisionID --> cpArbiter
    
    cpArbiter --> Solver["Impulse solver"]
    cpArbiter --> Callbacks["Collision callbacks"]
```

The system maintains collision IDs for temporal coherence, allowing it to reuse previous frame calculations as starting points for GJK, significantly improving performance for persistent contacts.

**Sources:** [src/cpCollision.c:701-726](), [src/cpCollision.c:457-467]()18:T3029,# Rigid Bodies (cpBody)

<details>
<summary>Relevant source files</summary>

The following files were used as context for generating this wiki page:

- [include/chipmunk/cpBody.h](include/chipmunk/cpBody.h)
- [src/cpBody.c](src/cpBody.c)
- [src/cpSpaceComponent.c](src/cpSpaceComponent.c)

</details>



This document covers the `cpBody` system in Chipmunk2D, which represents rigid bodies in the physics simulation. Rigid bodies are the fundamental moving objects that have mass, position, velocity, and other physical properties. For information about collision shapes that define the geometry of bodies, see [Collision Shapes](#2.4). For details about the overall physics simulation pipeline, see [Physics Simulation (cpSpace)](#2.1).

## Body Types

Chipmunk2D supports three distinct body types that determine how a body participates in physics simulation:

```mermaid
graph TB
    subgraph BodyTypes["cpBodyType Classification"]
        Dynamic["CP_BODY_TYPE_DYNAMIC<br/>• Affected by forces/gravity<br/>• Finite mass and moment<br/>• Full collision response<br/>• Can sleep"]
        
        Kinematic["CP_BODY_TYPE_KINEMATIC<br/>• Infinite mass<br/>• User-controlled movement<br/>• Not affected by forces<br/>• Cannot sleep"]
        
        Static["CP_BODY_TYPE_STATIC<br/>• Never moves<br/>• Infinite mass<br/>• Optimized collision detection<br/>• Always 'sleeping'"]
    end
    
    subgraph Interactions["Collision Interactions"]
        DynDyn["Dynamic ↔ Dynamic<br/>Full collision response"]
        DynKin["Dynamic ↔ Kinematic<br/>Only dynamic affected"]
        DynStat["Dynamic ↔ Static<br/>Only dynamic affected"]
        KinKin["Kinematic ↔ Kinematic<br/>Callbacks only"]
        KinStat["Kinematic ↔ Static<br/>Callbacks only"]
        StatStat["Static ↔ Static<br/>No callbacks"]
    end
    
    Dynamic --> DynDyn
    Dynamic --> DynKin
    Dynamic --> DynStat
    Kinematic --> KinKin
    Kinematic --> KinStat
    Static --> StatStat
```

**Sources:** [include/chipmunk/cpBody.h:28-41](), [src/cpBody.c:136-146](), [src/cpBody.c:148-201]()

## Core Properties and State

A `cpBody` maintains several categories of physical state that define its behavior in the simulation:

| Property Category | Fields | Description |
|------------------|--------|-------------|
| **Mass Properties** | `m`, `i`, `m_inv`, `i_inv`, `cog` | Mass, moment of inertia, inverses, center of gravity |
| **Position State** | `p`, `a`, `transform` | Position, angle, transformation matrix |
| **Velocity State** | `v`, `w`, `v_bias`, `w_bias` | Linear/angular velocity, bias velocities for solver |
| **Force Accumulation** | `f`, `t` | Accumulated forces and torques for next integration |
| **Relationships** | `shapeList`, `arbiterList`, `constraintList` | Linked lists of attached objects |
| **Sleeping State** | `sleeping.root`, `sleeping.next`, `sleeping.idleTime` | Component-based sleeping system |

```mermaid
graph TB
    subgraph BodyState["cpBody State Management"]
        MassProps["Mass Properties<br/>• body->m (mass)<br/>• body->i (moment)<br/>• body->m_inv, body->i_inv<br/>• body->cog (center of gravity)"]
        
        Transform["Transform State<br/>• body->p (position)<br/>• body->a (angle)<br/>• body->transform (matrix)"]
        
        Dynamics["Dynamic State<br/>• body->v (velocity)<br/>• body->w (angular velocity)<br/>• body->f (force)<br/>• body->t (torque)"]
        
        Sleep["Sleeping State<br/>• body->sleeping.root<br/>• body->sleeping.next<br/>• body->sleeping.idleTime"]
    end
    
    subgraph Integration["Integration Pipeline"]
        VelUpdate["cpBodyUpdateVelocity<br/>Apply forces to velocity"]
        PosUpdate["cpBodyUpdatePosition<br/>Apply velocity to position"]
    end
    
    Dynamics --> VelUpdate
    VelUpdate --> PosUpdate
    PosUpdate --> Transform
    MassProps --> VelUpdate
```

**Sources:** [src/cpBody.c:34-66](), [src/cpBody.c:494-522]()

## Lifecycle Management

Bodies follow a standard allocation, initialization, and destruction pattern:

```mermaid
flowchart TD
    Alloc["cpBodyAlloc()<br/>Allocate memory"]
    Init["cpBodyInit(body, mass, moment)<br/>Initialize with mass properties"]
    SetType["cpBodySetType(body, type)<br/>Optional: change body type"]
    
    NewDynamic["cpBodyNew(mass, moment)<br/>Create dynamic body"]
    NewKinematic["cpBodyNewKinematic()<br/>Create kinematic body"]
    NewStatic["cpBodyNewStatic()<br/>Create static body"]
    
    AddToSpace["Add to cpSpace<br/>Begin simulation"]
    
    Destroy["cpBodyDestroy(body)<br/>Cleanup (no-op)"]
    Free["cpBodyFree(body)<br/>Deallocate memory"]
    
    %% Manual path
    Alloc --> Init
    Init --> SetType
    SetType --> AddToSpace
    
    %% Convenience constructors
    NewDynamic --> AddToSpace
    NewKinematic --> AddToSpace  
    NewStatic --> AddToSpace
    
    %% Cleanup
    AddToSpace --> Destroy
    Destroy --> Free
```

**Sources:** [src/cpBody.c:27-101]()

## Mass Properties

Bodies automatically calculate their mass properties from attached shapes when they are dynamic bodies. This system ensures the center of gravity and moment of inertia remain consistent as shapes are added or removed:

```mermaid
graph TB
    subgraph MassSystem["Mass Property System"]
        ShapeAdd["cpBodyAddShape(body, shape)<br/>Add shape to body->shapeList"]
        ShapeRemove["cpBodyRemoveShape(body, shape)<br/>Remove from body->shapeList"]
        
        MassCalc["cpBodyAccumulateMassFromShapes(body)<br/>Recalculate mass properties"]
        
        Properties["Mass Properties Updated<br/>• body->m (total mass)<br/>• body->i (total moment)<br/>• body->cog (center of gravity)<br/>• body->m_inv, body->i_inv (inverses)"]
    end
    
    subgraph Conditions["When Mass Recalculation Occurs"]
        DynamicOnly["Only for CP_BODY_TYPE_DYNAMIC"]
        HasMass["Only if shape->massInfo.m > 0"]
    end
    
    ShapeAdd --> MassCalc
    ShapeRemove --> MassCalc
    MassCalc --> Properties
    
    DynamicOnly --> MassCalc
    HasMass --> MassCalc
```

**Sources:** [src/cpBody.c:206-239](), [src/cpBody.c:289-324](), [src/cpBody.c:254-280]()

## Physics Integration

Bodies use a two-phase integration system where velocity is updated first using forces, then position is updated using the new velocity:

```mermaid
flowchart TD
    subgraph VelocityIntegration["cpBodyUpdateVelocity Phase"]
        Gravity["Apply gravity:<br/>body->v += gravity * dt"]
        Forces["Apply forces:<br/>body->v += body->f * body->m_inv * dt"]
        Damping["Apply damping:<br/>body->v *= damping"]
        AngularForces["Apply torque:<br/>body->w += body->t * body->i_inv * dt"]
        ResetForces["Reset forces:<br/>body->f = 0, body->t = 0"]
    end
    
    subgraph PositionIntegration["cpBodyUpdatePosition Phase"]
        LinearPos["Update position:<br/>body->p += (body->v + body->v_bias) * dt"]
        AngularPos["Update angle:<br/>body->a += (body->w + body->w_bias) * dt"]
        UpdateTransform["SetTransform(body, body->p, body->a)<br/>Update transformation matrix"]
        ResetBias["Reset bias velocities:<br/>body->v_bias = 0, body->w_bias = 0"]
    end
    
    Gravity --> Forces
    Forces --> Damping  
    Damping --> AngularForces
    AngularForces --> ResetForces
    
    ResetForces --> LinearPos
    LinearPos --> AngularPos
    AngularPos --> UpdateTransform
    UpdateTransform --> ResetBias
```

**Sources:** [src/cpBody.c:494-522](), [src/cpBody.c:347-382]()

## Sleeping System

The sleeping system groups connected bodies into components and puts entire components to sleep when their kinetic energy falls below thresholds. This optimization skips physics calculations for stationary objects:

```mermaid
graph TB
    subgraph SleepingComponents["Component-Based Sleeping"]
        Root["Component Root<br/>body->sleeping.root"]
        LinkedList["Linked List<br/>body->sleeping.next"]
        IdleTime["Idle Timer<br/>body->sleeping.idleTime"]
    end
    
    subgraph ComponentDetection["cpSpaceProcessComponents()"]
        FloodFill["FloodFillComponent()<br/>Group connected bodies"]
        EnergyCheck["ComponentActive()<br/>Check kinetic energy"]
        Deactivate["cpSpaceDeactivateBody()<br/>Remove from simulation"]
    end
    
    subgraph WakeSystem["Activation System"]
        Activate["cpBodyActivate(body)<br/>Wake component"]
        ActivateStatic["cpBodyActivateStatic()<br/>Wake touching bodies"]
        SpaceActivate["cpSpaceActivateBody()<br/>Add back to simulation"]
    end
    
    Root --> LinkedList
    LinkedList --> IdleTime
    
    FloodFill --> EnergyCheck
    EnergyCheck --> Deactivate
    
    Activate --> SpaceActivate
    ActivateStatic --> Activate
```

**Sources:** [src/cpSpaceComponent.c:221-307](), [src/cpSpaceComponent.c:120-167](), [src/cpSpaceComponent.c:28-80]()

## Coordinate Transformations

Bodies provide transformation functions between local body coordinates and world coordinates:

| Function | Purpose | Implementation |
|----------|---------|----------------|
| `cpBodyLocalToWorld()` | Convert body-local point to world coordinates | `cpTransformPoint(body->transform, point)` |
| `cpBodyWorldToLocal()` | Convert world point to body-local coordinates | `cpTransformPoint(cpTransformRigidInverse(body->transform), point)` |
| `cpBodyGetPosition()` | Get body position in world coordinates | `cpTransformPoint(body->transform, cpvzero)` |
| `cpBodySetPosition()` | Set body position, updates transform | Updates `body->p` and calls `SetTransform()` |

**Sources:** [src/cpBody.c:525-534](), [src/cpBody.c:368-382](), [src/cpBody.c:347-357]()

## Force and Impulse Application

Bodies support applying forces and impulses at arbitrary points, which automatically generates appropriate torques:

```mermaid
graph LR
    subgraph ForceApplication["Force Application"]
        WorldForce["cpBodyApplyForceAtWorldPoint()<br/>force + point in world coords"]
        LocalForce["cpBodyApplyForceAtLocalPoint()<br/>force + point in body coords"]
        
        ForceCalc["Calculations:<br/>• body->f += force<br/>• r = point - center_of_gravity<br/>• body->t += cross(r, force)"]
    end
    
    subgraph ImpulseApplication["Impulse Application"]  
        WorldImpulse["cpBodyApplyImpulseAtWorldPoint()<br/>impulse + point in world coords"]
        LocalImpulse["cpBodyApplyImpulseAtLocalPoint()<br/>impulse + point in body coords"]
        
        ImpulseCalc["apply_impulse(body, impulse, r)<br/>Immediate velocity change"]
    end
    
    WorldForce --> ForceCalc
    LocalForce --> WorldForce
    
    WorldImpulse --> ImpulseCalc
    LocalImpulse --> WorldImpulse
```

**Sources:** [src/cpBody.c:536-565]()

## Body Relationships

Bodies maintain linked lists of attached shapes, active collision arbiters, and constraints. These relationships are managed automatically by the space:

| Relationship Type | List Field | Iterator Function | Purpose |
|------------------|------------|------------------|---------|
| **Shapes** | `body->shapeList` | `cpBodyEachShape()` | Collision geometry attached to body |
| **Arbiters** | `body->arbiterList` | `cpBodyEachArbiter()` | Active collision contacts |
| **Constraints** | `body->constraintList` | `cpBodyEachConstraint()` | Joints and springs attached to body |

```mermaid
graph TB
    subgraph BodyRelationships["cpBody Relationship Management"]
        Body["cpBody"]
        
        ShapeList["body->shapeList<br/>Doubly-linked list of cpShape*"]
        ArbiterList["body->arbiterList<br/>Linked list of cpArbiter*"]
        ConstraintList["body->constraintList<br/>Linked list of cpConstraint*"]
        
        Body --> ShapeList
        Body --> ArbiterList  
        Body --> ConstraintList
    end
    
    subgraph IteratorFunctions["Iterator Functions"]
        EachShape["cpBodyEachShape(body, func, data)<br/>Iterate over attached shapes"]
        EachArbiter["cpBodyEachArbiter(body, func, data)<br/>Iterate over active collisions"] 
        EachConstraint["cpBodyEachConstraint(body, func, data)<br/>Iterate over attached joints"]
    end
    
    ShapeList --> EachShape
    ArbiterList --> EachArbiter
    ConstraintList --> EachConstraint
```

**Sources:** [src/cpBody.c:591-626](), [src/cpBody.c:289-324]()19:T340b

