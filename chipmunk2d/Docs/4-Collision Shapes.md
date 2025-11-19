# Collision Shapes

<details>
<summary>Relevant source files</summary>

The following files were used as context for generating this wiki page:

- [include/chipmunk/cpPolyShape.h](include/chipmunk/cpPolyShape.h)
- [include/chipmunk/cpShape.h](include/chipmunk/cpShape.h)
- [src/cpPolyShape.c](src/cpPolyShape.c)
- [src/cpShape.c](src/cpShape.c)

</details>



This document covers the collision geometry system in Chipmunk2D, including the shape type hierarchy, shape properties, spatial queries, and collision filtering. Collision shapes define the geometric boundaries of rigid bodies for collision detection and response.

For information about collision detection algorithms that operate on these shapes, see [Collision Detection System](#2.2). For details about the rigid bodies that shapes attach to, see [Rigid Bodies (cpBody)](#2.3).

## Shape Type Architecture

The collision shape system uses a polymorphic class-based design implemented in C, with three primary shape types that inherit from a common base.

### Shape Class Hierarchy

```mermaid
classDiagram
    class cpShape {
        +cpShapeClass* klass
        +cpBody* body
        +cpShapeMassInfo massInfo
        +cpBB bb
        +cpBool sensor
        +cpFloat e
        +cpFloat u
        +cpVect surfaceV
        +cpShapeFilter filter
        +cpCollisionType type
        +cpDataPointer userData
        +pointQuery()
        +segmentQuery()
        +cacheData()
    }
    
    class cpShapeClass {
        +cpShapeType type
        +cpShapeCacheDataImpl cacheData
        +cpShapeDestroyImpl destroy
        +cpShapePointQueryImpl pointQuery
        +cpShapeSegmentQueryImpl segmentQuery
    }
    
    class cpCircleShape {
        +cpVect c
        +cpFloat r
        +cpVect tc
    }
    
    class cpSegmentShape {
        +cpVect a
        +cpVect b
        +cpVect n
        +cpFloat r
        +cpVect ta
        +cpVect tb
        +cpVect tn
        +cpVect a_tangent
        +cpVect b_tangent
    }
    
    class cpPolyShape {
        +int count
        +cpSplittingPlane* planes
        +cpFloat r
        +cpSplittingPlane _planes[CP_POLY_SHAPE_INLINE_ALLOC]
    }
    
    cpShape *-- cpShapeClass
    cpCircleShape --|> cpShape
    cpSegmentShape --|> cpShape
    cpPolyShape --|> cpShape
    
    class cpSplittingPlane {
        +cpVect v0
        +cpVect n
    }
    
    cpPolyShape *-- cpSplittingPlane
```

Sources: [src/cpShape.c:31-58](), [include/chipmunk/cpShape.h:22-24](), [src/cpPolyShape.c:25-29]()

### Shape Polymorphism Implementation

Each shape type implements a virtual method table through the `cpShapeClass` structure, enabling type-specific behavior while maintaining a common interface.

```mermaid
flowchart TD
    ShapeCall["cpShapePointQuery()"] --> Dispatch["shape->klass->pointQuery"]
    
    Dispatch --> CircleImpl["cpCircleShapePointQuery()"]
    Dispatch --> SegmentImpl["cpSegmentShapePointQuery()"]
    Dispatch --> PolyImpl["cpPolyShapePointQuery()"]
    
    CircleImpl --> CircleResult["Circle distance calculation"]
    SegmentImpl --> SegmentResult["Segment closest point"]
    PolyImpl --> PolyResult["Polygon edge iteration"]
```

Sources: [src/cpShape.c:222-234](), [src/cpShape.c:332-338](), [src/cpShape.c:479-485](), [src/cpPolyShape.c:182-188]()

## Core Shape Properties

All shapes share a common set of physical and behavioral properties that affect collision detection and response.

### Physical Properties

| Property | Type | Description | Default |
|----------|------|-------------|---------|
| `mass` | `cpFloat` | Mass for inertia calculations | 0.0 |
| `density` | `cpFloat` | Mass per unit area | Calculated |
| `moment` | `cpFloat` | Moment of inertia | Calculated |
| `area` | `cpFloat` | Surface area | Calculated |
| `centerOfGravity` | `cpVect` | Center of mass offset | Calculated |

### Material Properties

| Property | Type | Description | Range |
|----------|------|-------------|-------|
| `elasticity` (e) | `cpFloat` | Restitution coefficient | ≥ 0.0 |
| `friction` (u) | `cpFloat` | Surface friction | ≥ 0.0 |
| `surfaceVelocity` | `cpVect` | Surface conveyor velocity | Any |

### Behavioral Properties

| Property | Type | Description |
|----------|------|-------------|
| `sensor` | `cpBool` | Detects collisions without response |
| `collisionType` | `cpCollisionType` | User-defined collision category |
| `filter` | `cpShapeFilter` | Collision filtering parameters |
| `userData` | `cpDataPointer` | User-defined data pointer |

Sources: [src/cpShape.c:94-110](), [src/cpShape.c:131-170](), [include/chipmunk/cpShape.h:106-159]()

## Shape Types

### Circle Shapes

Circle shapes represent perfect circles with a center offset and radius.

```mermaid
graph LR
    subgraph "cpCircleShape Structure"
        Center["c: cpVect<br/>(local center)"]
        Radius["r: cpFloat<br/>(radius)"]
        TransCenter["tc: cpVect<br/>(transformed center)"]
    end
    
    subgraph "Operations"
        Create["cpCircleShapeNew()"]
        Query["cpCircleShapePointQuery()"]
        Update["cpCircleShapeCacheData()"]
    end
    
    Create --> Center
    Create --> Radius
    Update --> TransCenter
    Query --> TransCenter
    Query --> Radius
```

**Key Functions:**
- `cpCircleShapeNew(body, radius, offset)` - Creates a new circle shape
- `cpCircleShapeGetRadius()` / `cpCircleShapeGetOffset()` - Property accessors
- `cpCircleShapeCacheData()` - Updates bounding box and transformed center

Sources: [src/cpShape.c:285-369](), [include/chipmunk/cpShape.h:163-176]()

### Segment Shapes

Segment shapes represent line segments with rounded endpoints, effectively creating capsule-like collision boundaries.

```mermaid
graph TD
    subgraph "cpSegmentShape Structure"
        EndpointA["a: cpVect<br/>(endpoint A)"]
        EndpointB["b: cpVect<br/>(endpoint B)"]
        Normal["n: cpVect<br/>(normal vector)"]
        Radius["r: cpFloat<br/>(thickness)"]
        TransA["ta: cpVect<br/>(transformed A)"]
        TransB["tb: cpVect<br/>(transformed B)"]
        TransN["tn: cpVect<br/>(transformed normal)"]
    end
    
    subgraph "Neighbor Support"
        ATangent["a_tangent: cpVect"]
        BTangent["b_tangent: cpVect"]
        Neighbors["cpSegmentShapeSetNeighbors()"]
    end
    
    Neighbors --> ATangent
    Neighbors --> BTangent
```

**Neighbor System:** Segments can be linked to adjacent segments to prevent collision artifacts at joints, particularly useful for terrain generation.

Sources: [src/cpShape.c:372-547](), [include/chipmunk/cpShape.h:178-198]()

### Polygon Shapes

Polygon shapes represent convex polygons with optional rounded corners. They use a splitting plane representation for efficient collision detection.

```mermaid
graph TB
    subgraph "cpPolyShape Structure"
        Count["count: int<br/>(vertex count)"]
        Planes["planes: cpSplittingPlane*<br/>(edge data)"]
        Radius["r: cpFloat<br/>(corner radius)"]
        Inline["_planes[CP_POLY_SHAPE_INLINE_ALLOC]<br/>(small polygon optimization)"]
    end
    
    subgraph "cpSplittingPlane"
        Vertex["v0: cpVect<br/>(vertex position)"]
        PlaneNormal["n: cpVect<br/>(edge normal)"]
    end
    
    subgraph "Specialized Constructors"
        PolyNew["cpPolyShapeNew()"]
        BoxNew["cpBoxShapeNew()"]
        ConvexHull["cpConvexHull()"]
    end
    
    Planes --> Vertex
    Planes --> PlaneNormal
    PolyNew --> ConvexHull
    BoxNew --> Planes
```

**Memory Optimization:** Small polygons use inline storage (`_planes` array) to avoid heap allocation, while larger polygons dynamically allocate memory.

**Convex Hull:** The `cpPolyShapeNew()` function automatically computes a convex hull from input vertices, while `cpPolyShapeNewRaw()` assumes vertices are already convex.

Sources: [src/cpPolyShape.c:25-257](), [include/chipmunk/cpPolyShape.h:22-56]()

## Spatial Queries

Shapes support two primary spatial query operations for collision detection and game logic.

### Point Queries

Point queries determine the closest point on a shape's surface to a given point and return distance information.

```mermaid
flowchart TD
    Point["Query Point p"] --> ShapeQuery["cpShapePointQuery(shape, p, info)"]
    
    ShapeQuery --> CircleCalc["Circle: Distance to center"]
    ShapeQuery --> SegmentCalc["Segment: Closest point on line"]
    ShapeQuery --> PolyCalc["Polygon: Check all edges"]
    
    CircleCalc --> Result["cpPointQueryInfo"]
    SegmentCalc --> Result
    PolyCalc --> Result
    
    subgraph "cpPointQueryInfo"
        Shape["shape: cpShape*"]
        ClosestPoint["point: cpVect"]
        Distance["distance: cpFloat"]
        Gradient["gradient: cpVect"]
    end
    
    Result --> Shape
    Result --> ClosestPoint
    Result --> Distance
    Result --> Gradient
```

**Distance Semantics:** Negative distances indicate the point is inside the shape, positive distances indicate outside.

Sources: [src/cpShape.c:222-234](), [src/cpShape.c:299-312](), [src/cpShape.c:408-423](), [src/cpPolyShape.c:67-103]()

### Segment Queries

Segment queries perform ray/line casting against shapes to find intersection points.

```mermaid
sequenceDiagram
    participant Caller
    participant cpShape
    participant ShapeImpl
    participant Info as cpSegmentQueryInfo
    
    Caller->>cpShape: cpShapeSegmentQuery(shape, a, b, radius)
    cpShape->>cpShape: Check if start point inside (pointQuery)
    
    alt Start point inside
        cpShape->>Info: Set alpha=0, immediate hit
    else Start point outside  
        cpShape->>ShapeImpl: Call shape-specific segmentQuery
        ShapeImpl->>ShapeImpl: Calculate intersection
        ShapeImpl->>Info: Set intersection details
    end
    
    cpShape->>Caller: Return hit boolean
```

**Segment Query Info:**
- `shape` - The intersected shape (NULL if no hit)
- `point` - The intersection point in world coordinates
- `normal` - The surface normal at intersection
- `alpha` - Normalized distance along segment [0,1]

Sources: [src/cpShape.c:237-257](), [include/chipmunk/cpShape.h:39-49]()

## Shape Filtering and Collision Detection

### Collision Filtering

The `cpShapeFilter` system provides fast collision filtering before expensive collision detection.

```mermaid
graph TD
    subgraph "cpShapeFilter"
        Group["group: cpGroup<br/>(collision group)"]
        Categories["categories: cpBitmask<br/>(what I am)"]
        Mask["mask: cpBitmask<br/>(what I collide with)"]
    end
    
    subgraph "Filter Logic"
        GroupCheck["Same non-zero group?"]
        CategoryCheck["(a.categories & b.mask) &&<br/>(b.categories & a.mask)"]
    end
    
    GroupCheck -->|Yes| NoCollision["No Collision"]
    GroupCheck -->|No| CategoryCheck
    CategoryCheck -->|True| Collision["Collision Allowed"]
    CategoryCheck -->|False| NoCollision
    
    subgraph "Predefined Filters"
        FilterAll["CP_SHAPE_FILTER_ALL"]
        FilterNone["CP_SHAPE_FILTER_NONE"]
    end
```

**Group System:** Objects with the same non-zero group value never collide, useful for composite objects.

**Category/Mask System:** Bitwise filtering where both objects must agree to collide:
- `categories` - Defines what collision categories this object belongs to
- `mask` - Defines what categories this object can collide with

Sources: [include/chipmunk/cpShape.h:51-75](), [src/cpShape.c:197-208]()

### Shape-to-Shape Collision

The `cpShapesCollide()` function provides direct collision testing between shapes, bypassing the full physics simulation.

```mermaid
flowchart LR
    ShapeA["Shape A"] --> Collide["cpShapesCollide(a, b)"]
    ShapeB["Shape B"] --> Collide
    
    Collide --> CollisionInfo["cpCollide() internal"]
    CollisionInfo --> ContactSet["cpContactPointSet"]
    
    subgraph "cpContactPointSet"
        Count["count: int"]
        Normal["normal: cpVect"]
        Points["points[]: cpContactPoint[]"]
    end
    
    subgraph "cpContactPoint"
        PointA["pointA: cpVect"]
        PointB["pointB: cpVect"]
        Distance["distance: cpFloat"]
    end
    
    ContactSet --> Count
    ContactSet --> Normal
    ContactSet --> Points
    Points --> PointA
    Points --> PointB
    Points --> Distance
```

Sources: [src/cpShape.c:259-283](), [include/chipmunk/cpShape.h:94-95]()

## Memory Management and Lifecycle

### Shape Creation Pattern

Chipmunk uses a consistent allocation pattern across all shape types:

1. **Alloc** - `cpXxxShapeAlloc()` allocates memory
2. **Init** - `cpXxxShapeInit()` initializes the structure  
3. **New** - `cpXxxShapeNew()` combines alloc + init

### Bounding Box Caching

Shapes cache their axis-aligned bounding boxes to optimize broad-phase collision detection.

```mermaid
sequenceDiagram
    participant Body
    participant Shape
    participant SpatialIndex
    
    Body->>Body: Transform updated
    Body->>Shape: cpShapeUpdate(transform)
    Shape->>Shape: klass->cacheData()
    Shape->>Shape: Update bb field
    Shape->>SpatialIndex: Reindex if in space
    
    Note over Shape: Cached bb used for<br/>broad-phase optimization
```

**Cache Lifecycle:**
- Updated when body transform changes
- Used by spatial indexing structures
- Includes shape radius for swept bounds

Sources: [src/cpShape.c:210-220](), [src/cpShape.c:291-296](), [src/cpShape.c:378-405](), [src/cpPolyShape.c:40-64]()1a:T3101,# Spatial Indexing and Optimization

<details>
<summary>Relevant source files</summary>

The following files were used as context for generating this wiki page:

- [include/chipmunk/cpSpatialIndex.h](include/chipmunk/cpSpatialIndex.h)
- [src/cpArray.c](src/cpArray.c)
- [src/cpBBTree.c](src/cpBBTree.c)
- [src/cpHashSet.c](src/cpHashSet.c)
- [src/cpSpaceHash.c](src/cpSpaceHash.c)
- [src/cpSweep1D.c](src/cpSweep1D.c)

</details>



## Purpose and Scope

This document covers Chipmunk2D's spatial indexing system, which provides broad-phase collision detection optimizations to accelerate physics simulation performance. The spatial index implementations efficiently determine which objects might potentially collide before performing expensive narrow-phase collision tests.

For information about the narrow-phase collision detection algorithms that follow spatial indexing, see [Collision Detection System](#2.2). For the overall physics simulation pipeline, see [Physics Simulation (cpSpace)](#2.1).

## Overview

Spatial indexing is a critical optimization technique that divides 2D space into regions and tracks which objects occupy each region. This allows the physics engine to quickly eliminate object pairs that are too far apart to possibly collide, reducing collision detection complexity from O(n²) to approximately O(n log n) or better.

### Spatial Index Architecture

```mermaid
classDiagram
    class cpSpatialIndex {
        +cpSpatialIndexClass* klass
        +cpSpatialIndexBBFunc bbfunc
        +cpSpatialIndex* staticIndex
        +cpSpatialIndex* dynamicIndex
        +insert(obj, hashid)
        +remove(obj, hashid)
        +query(obj, bb, func, data)
        +reindexQuery(func, data)
    }
    
    class cpSpatialIndexClass {
        +destroy()
        +count()
        +insert()
        +remove()
        +query()
        +segmentQuery()
        +reindex()
        +reindexObject()
        +reindexQuery()
    }
    
    class cpSpaceHash {
        +cpFloat celldim
        +int numcells
        +cpSpaceHashBin** table
        +cpHashSet* handleSet
        +cpTimestamp stamp
    }
    
    class cpBBTree {
        +cpBBTreeVelocityFunc velocityFunc
        +cpHashSet* leaves
        +Node* root
        +Node* pooledNodes
        +cpTimestamp stamp
    }
    
    class cpSweep1D {
        +int num
        +int max
        +TableCell* table
    }
    
    cpSpatialIndex <|-- cpSpaceHash
    cpSpatialIndex <|-- cpBBTree
    cpSpatialIndex <|-- cpSweep1D
    cpSpatialIndex --> cpSpatialIndexClass
```

Sources: [include/chipmunk/cpSpatialIndex.h:54-64](), [src/cpSpaceHash.c:28-42](), [src/cpBBTree.c:32-44](), [src/cpSweep1D.c:37-44]()

## Spatial Index Interface

The `cpSpatialIndex` provides a polymorphic interface that all spatial index implementations must support. This allows the physics engine to switch between different spatial indexing strategies transparently.

### Core Operations

| Operation | Purpose | Usage |
|-----------|---------|--------|
| `insert` | Add object to index | Called when objects are created or moved significantly |
| `remove` | Remove object from index | Called when objects are destroyed |
| `query` | Find objects overlapping a bounding box | Used for collision detection and spatial queries |
| `reindexQuery` | Update all object positions and find overlaps | Called each physics step |
| `segmentQuery` | Find objects intersecting a line segment | Used for raycasting |

### Function Pointer Interface

```mermaid
graph TB
    subgraph "cpSpatialIndexClass Virtual Table"
        destroy["destroy()"]
        insert["insert()"]
        remove["remove()"]
        query["query()"]
        reindexQuery["reindexQuery()"]
        segmentQuery["segmentQuery()"]
    end
    
    subgraph "cpSpaceHash Implementation"
        cpSpaceHashInsert["cpSpaceHashInsert()"]
        cpSpaceHashRemove["cpSpaceHashRemove()"]
        cpSpaceHashQuery["cpSpaceHashQuery()"]
        cpSpaceHashReindexQuery["cpSpaceHashReindexQuery()"]
    end
    
    subgraph "cpBBTree Implementation"
        cpBBTreeInsert["cpBBTreeInsert()"]
        cpBBTreeRemove["cpBBTreeRemove()"]
        cpBBTreeQuery["cpBBTreeQuery()"]
        cpBBTreeReindexQuery["cpBBTreeReindexQuery()"]
    end
    
    insert --> cpSpaceHashInsert
    insert --> cpBBTreeInsert
    remove --> cpSpaceHashRemove
    remove --> cpBBTreeRemove
    query --> cpSpaceHashQuery
    query --> cpBBTreeQuery
    reindexQuery --> cpSpaceHashReindexQuery
    reindexQuery --> cpBBTreeReindexQuery
```

Sources: [include/chipmunk/cpSpatialIndex.h:133-149](), [src/cpSpaceHash.c:573-591](), [src/cpBBTree.c:709-727]()

## Spatial Hash Implementation

The `cpSpaceHash` divides space into a uniform grid of cells, where each cell can contain multiple objects. Objects that span multiple cells are stored in all relevant cells.

### Internal Structure

```mermaid
graph TB
    subgraph "cpSpaceHash Structure"
        spatialIndex["cpSpatialIndex base"]
        celldim["cpFloat celldim<br/>Cell dimensions"]
        numcells["int numcells<br/>Hash table size"]
        table["cpSpaceHashBin** table<br/>Hash table array"]
        handleSet["cpHashSet* handleSet<br/>Object handle storage"]
        stamp["cpTimestamp stamp<br/>Query stamp counter"]
    end
    
    subgraph "Hash Table Buckets"
        bin1["cpSpaceHashBin<br/>handle: obj1<br/>next: bin2"]
        bin2["cpSpaceHashBin<br/>handle: obj2<br/>next: NULL"]
        bin3["cpSpaceHashBin<br/>handle: obj3<br/>next: NULL"]
    end
    
    subgraph "Handle Management"
        handle1["cpHandle<br/>obj: shape1<br/>retain: 2<br/>stamp: 123"]
        handle2["cpHandle<br/>obj: shape2<br/>retain: 1<br/>stamp: 124"]
    end
    
    table --> bin1
    table --> bin3
    bin1 --> bin2
    bin1 --> handle1
    bin2 --> handle2
    bin3 --> handle2
```

Sources: [src/cpSpaceHash.c:28-42](), [src/cpSpaceHash.c:47-51](), [src/cpSpaceHash.c:96-99]()

### Hash Function and Cell Mapping

The spatial hash uses a simple but effective hash function to map 2D grid coordinates to hash table indices:

```mermaid
flowchart TD
    bb["Object Bounding Box<br/>(l, b, r, t)"]
    
    coords["Grid Coordinates<br/>l_cell = floor(bb.l / celldim)<br/>r_cell = floor(bb.r / celldim)<br/>b_cell = floor(bb.b / celldim)<br/>t_cell = floor(bb.t / celldim)"]
    
    hash["Hash Function<br/>hash_func(x, y, n)<br/>= (x*1640531513 ^ y*2654435789) % n"]
    
    cells["For each cell (i, j)<br/>where l_cell ≤ i ≤ r_cell<br/>and b_cell ≤ j ≤ t_cell"]
    
    insert["Insert object handle<br/>into table[hash_func(i, j, numcells)]"]
    
    bb --> coords
    coords --> cells
    cells --> hash
    hash --> insert
```

Sources: [src/cpSpaceHash.c:242-268](), [src/cpSpaceHash.c:226-239]()

### Query Process

The spatial hash query process efficiently finds potential collision pairs:

1. **Stamp-based Duplicate Prevention**: Uses timestamps to avoid testing the same pair multiple times
2. **Grid Cell Iteration**: Only checks cells that overlap the query bounding box  
3. **Handle Reference Counting**: Manages object lifetime during queries

Sources: [src/cpSpaceHash.c:376-397](), [src/cpSpaceHash.c:354-374]()

## Bounding Box Tree Implementation

The `cpBBTree` uses a hierarchical tree structure where each internal node contains a bounding box that encompasses all objects in its subtree. This provides logarithmic query performance.

### Tree Structure

```mermaid
graph TD
    subgraph "cpBBTree Node Hierarchy"
        root["Root Node<br/>bb: (0,0,100,100)<br/>obj: NULL"]
        
        internal1["Internal Node<br/>bb: (0,0,50,100)<br/>obj: NULL"]
        internal2["Internal Node<br/>bb: (50,0,100,100)<br/>obj: NULL"]
        
        leaf1["Leaf Node<br/>bb: (10,10,20,20)<br/>obj: shape1"]
        leaf2["Leaf Node<br/>bb: (30,30,40,40)<br/>obj: shape2"]
        leaf3["Leaf Node<br/>bb: (60,60,70,70)<br/>obj: shape3"]
        leaf4["Leaf Node<br/>bb: (80,80,90,90)<br/>obj: shape4"]
    end
    
    subgraph "Node Structure"
        nodeStruct["Node {<br/>void* obj<br/>cpBB bb<br/>Node* parent<br/>union {<br/>  struct { Node* a, *b } children<br/>  struct { cpTimestamp stamp, Pair* pairs } leaf<br/>}<br/>}"]
    end
    
    root --> internal1
    root --> internal2
    internal1 --> leaf1
    internal1 --> leaf2
    internal2 --> leaf3
    internal2 --> leaf4
```

Sources: [src/cpBBTree.c:46-68](), [src/cpBBTree.c:271-284]()

### Tree Operations

The bounding box tree supports efficient insertion, removal, and rebalancing operations:

| Operation | Algorithm | Complexity |
|-----------|-----------|------------|
| Insert | Find best insertion point based on area increase | O(log n) |
| Remove | Remove node and rebalance parent chain | O(log n) |
| Query | Recursively descend tree, pruning non-overlapping subtrees | O(log n + k) |
| Rebalance | Top-down tree reconstruction using median splits | O(n log n) |

### Velocity-Based Prediction

The bounding box tree supports temporal coherence through velocity prediction, expanding bounding boxes based on object velocity to reduce the frequency of tree updates:

```mermaid
flowchart LR
    velocity["cpBBTreeVelocityFunc<br/>Returns object velocity"]
    expand["Expand Bounding Box<br/>bb = original_bb + velocity * dt"]
    reduce["Reduce Tree Updates<br/>Object can move within<br/>expanded bounds without<br/>requiring tree rebalancing"]
    
    velocity --> expand
    expand --> reduce
```

Sources: [src/cpBBTree.c:82-98](), [include/chipmunk/cpSpatialIndex.h:99-102]()

## 1D Sweep Implementation

The `cpSweep1D` implements a simple sweep-and-prune algorithm along a single axis. While less sophisticated than the other implementations, it provides good performance for scenarios with objects distributed primarily along one dimension.

### Structure and Algorithm

```mermaid
graph TB
    subgraph "cpSweep1D Structure"
        table["TableCell* table<br/>Dynamic array of objects"]
        num["int num<br/>Current object count"]
        max["int max<br/>Array capacity"]
    end
    
    subgraph "TableCell Structure"
        cell1["TableCell {<br/>void* obj: shape1<br/>Bounds bounds: {min: 10, max: 30}<br/>}"]
        cell2["TableCell {<br/>void* obj: shape2<br/>Bounds bounds: {min: 25, max: 45}<br/>}"]
        cell3["TableCell {<br/>void* obj: shape3<br/>Bounds bounds: {min: 50, max: 70}<br/>}"]
    end
    
    subgraph "Sweep Algorithm"
        sort["Sort by bounds.min"]
        sweep["For each object i:<br/>Check objects j where<br/>table[j].bounds.min < table[i].bounds.max"]
    end
    
    table --> cell1
    table --> cell2
    table --> cell3
    sort --> sweep
```

Sources: [src/cpSweep1D.c:32-44](), [src/cpSweep1D.c:205-233]()

## Performance Characteristics

The choice of spatial index depends on the specific characteristics of your simulation:

| Implementation | Best Use Case | Time Complexity | Memory Usage | Notes |
|----------------|---------------|-----------------|--------------|-------|
| `cpSpaceHash` | Uniform object distribution | O(1) average query | Medium | Simple, predictable performance |
| `cpBBTree` | Clustered objects, varying sizes | O(log n) query | Higher | Adaptive, supports velocity prediction |
| `cpSweep1D` | Simple scenes, debug builds | O(n) query | Lowest | Minimal overhead, no acceleration |

### Integration with Collision Detection

```mermaid
sequenceDiagram
    participant Space as "cpSpace"
    participant SpatialIndex as "Spatial Index"
    participant Collision as "Collision Detection"
    
    Space->>SpatialIndex: cpSpatialIndexReindexQuery()
    Note over SpatialIndex: Update object positions<br/>in spatial data structure
    
    loop For each potential collision pair
        SpatialIndex->>Space: Query callback with (objA, objB)
        Space->>Collision: cpSpaceCollideShapes(objA, objB)
        Note over Collision: Perform narrow-phase<br/>collision detection
        Collision-->>Space: Contact points or no collision
    end
    
    Space->>SpatialIndex: cpSpatialIndexCollideStatic()
    Note over Space: Collide dynamic objects<br/>against static spatial index
```

Sources: [src/cpSpaceHash.c:448-457](), [src/cpBBTree.c:639-654]()

The spatial indexing system integrates seamlessly with Chipmunk2D's collision detection pipeline, providing the critical broad-phase optimization that makes real-time physics simulation practical for complex scenes with many objects.

Sources: [src/cpSpaceHash.c](), [src/cpBBTree.c](), [src/cpSweep1D.c](), [include/chipmunk/cpSpatialIndex.h]()1b:T33f7

