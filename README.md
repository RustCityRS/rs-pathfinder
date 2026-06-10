<div align="center">
<pre>
██████╗ ██╗   ██╗███████╗████████╗ ██████╗██╗████████╗██╗   ██╗
██╔══██╗██║   ██║██╔════╝╚══██╔══╝██╔════╝██║╚══██╔══╝╚██╗ ██╔╝
██████╔╝██║   ██║███████╗   ██║   ██║     ██║   ██║    ╚████╔╝ 
██╔══██╗██║   ██║╚════██║   ██║   ██║     ██║   ██║     ╚██╔╝  
██║  ██║╚██████╔╝███████║   ██║   ╚██████╗██║   ██║      ██║   
╚═╝  ╚═╝ ╚═════╝ ╚══════╝   ╚═╝    ╚═════╝╚═╝   ╚═╝      ╚═╝   
</pre>
</div>

----

# rs-pathfinder — A BFS pathfinder & collision system

A high-performance Rust implementation of the rsmod pathfinding library (originally Kotlin). This crate implements
the full RuneScape server-side pathfinding, collision detection, line-of-sight, line-of-walk, reach checking, and
step validation systems with extensive optimizations including zero-allocation design, branchless arithmetic,
generation-based BFS resets, raw pointer traversal, and pre-allocated ring buffers for peak throughput.

The crate is published as lib name `rsmod`.

---

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Coordinate System](#coordinate-system)
- [Collision Flag Map](#collision-flag-map)
- [Collision Flags](#collision-flags)
- [Collision Strategies](#collision-strategies)
- [Pathfinder (BFS A*)](#pathfinder-bfs-a)
- [Naive Pathfinder](#naive-pathfinder)
- [Line of Sight / Line of Walk](#line-of-sight--line-of-walk)
- [Line Pathfinder (Ray Cast Path)](#line-pathfinder-ray-cast-path)
- [Step Validator](#step-validator)
- [Reach Strategy](#reach-strategy)
- [Rectangle Boundary](#rectangle-boundary)
- [Wall Collision Modification](#wall-collision-modification)
- [Location Types](#location-types)
- [Rotation Utilities](#rotation-utilities)
- [Global State and Safety Model](#global-state-and-safety-model)
- [Public API](#public-api)
- [Benchmark Harness](#benchmark-harness)

---

## Architecture Overview

```
lib.rs (public API + global state)
 └── rsmod/
      ├── pathfinder.rs          BFS pathfinder (size 1, 2, N)
      ├── naive_pathfinder.rs    Simple greedy NPC pathfinder
      ├── line_validator.rs      Line-of-sight / line-of-walk (bool)
      ├── line_pathfinder.rs     Line-of-sight / line-of-walk (returns path coords)
      ├── step_validator.rs      Single-step movement validation
      ├── collision/
      │    ├── collision.rs       CollisionFlagMap (zone-based tile storage)
      │    └── collision_strategy.rs  5 collision strategies (function pointers)
      ├── flag/
      │    ├── collision_flag.rs  All collision bitmask constants
      │    ├── block_flag.rs      BlockAccessFlag (N/E/S/W approach restrictions)
      │    └── direction_flag.rs  DirectionFlag (BFS backtrack direction encoding)
      ├── reach/
      │    ├── reach_strategy.rs  Reach dispatch (wall, wall decor, rectangle, exclusive)
      │    └── rectangle_boundary.rs  Adjacent-rectangle reach check for size 1 and N
      ├── utils/
      │    └── rotation.rs       Dimension/flag rotation for loc angles
      ├── coord_grid.rs          Packed coordinate (y, x, z) in a u32
      ├── line.rs                Constants and helpers for ray casting
      ├── loc_angle.rs           4-rotation enum for locs (West/North/East/South)
      ├── loc_layer.rs           4 loc layer types (Wall/WallDecor/Ground/GroundDecor)
      └── loc_shape.rs           23 loc shape types (walls, centrepieces, roofs, decor)
```

---

## Coordinate System

### CoordGrid (`coord_grid.rs`)

All coordinates are packed into a single `u32` for zero-allocation storage and return from pathfinders:

```
Bits  31-28  27-14  13-0
      y(2b)  x(14b) z(14b)
```

- **y** (level/floor): 2 bits, values 0-3. The game world has 4 vertical levels.
- **x**: 14 bits, values 0-16383. Absolute world X coordinate.
- **z**: 14 bits, values 0-16383. Absolute world Z coordinate (north-south axis, NOT height).

Construction: `CoordGrid::new(y, x, z)` packs via `(z & 0x3fff) | ((x & 0x3fff) << 14) | ((y & 0x3) << 28)`.

Extraction: `coord.y()`, `coord.x()`, `coord.z()` unpack with shifts and masks.

Waypoints returned by `find_path` and `line_of_sight`/`line_of_walk` are arrays of these packed `u32` values.

---

## Collision Flag Map

### CollisionFlagMap (`collision/collision.rs`)

The collision map is a zone-based sparse storage for per-tile collision flags across the entire game world.

**Zone model**: The world is divided into 8x8-tile **zones**. Each zone is identified by a zone index computed from the
tile coordinates and level:

```
zone_index(x, z, y) = ((x >> 3) & 0x7ff) | (((z >> 3) & 0x7ff) << 11) | ((y & 0x3) << 22)
```

This gives a maximum of `2048 * 2048 * 4 = 16,777,216` possible zones. Each zone, when allocated, stores a
`Box<[u32; 64]>` (8x8 = 64 tiles, each a u32 bitmask).

**Tile index within a zone**:

```
tile_index(x, z) = (x & 0x7) | ((z & 0x7) << 3)
```

**Lazy allocation**: Zones start as `None`. The first write to any tile in a zone allocates the full 64-entry array
initialized to `CollisionFlag::Open` (0x0). Reading an unallocated zone returns `CollisionFlag::Null` (0x7FFFFFFF),
which blocks all movement.

**Operations**:
| Method | Behavior |
|--------|----------|
| `get(x, z, y)` | Returns the flag bitmask, or `Null` if zone not allocated |
| `set(x, z, y, mask)` | Overwrites the tile flag (allocates zone if needed) |
| `add(x, z, y, mask)` | Bitwise OR the mask onto existing flags |
| `remove(x, z, y, mask)` | Bitwise AND-NOT the mask from existing flags |
| `is_flagged(x, z, y, masks)` | Returns true if any bit in `masks` is set on the tile |
| `allocate_if_absent(x, z, y)` | Ensures the zone exists |
| `deallocate_if_present(x, z, y)` | Drops the zone, tiles revert to `Null` |
| `is_zone_allocated(x, z, y)` | Checks if zone is allocated |

All operations use raw pointer arithmetic (`as_ptr().add(...)`) for performance, bypassing bounds checks.

---

## Collision Flags

### CollisionFlag (`flag/collision_flag.rs`)

A comprehensive `#[repr(u32)]` enum defining all tile collision bitmasks. The flags are layered into three tiers:

**Tier 1 - Movement flags** (bits 0-8, 0x1-0x100):
| Flag | Hex | Purpose |
|------|-----|---------|
| `Open` | 0x0 | No collision |
| `WallNorthWest` | 0x1 | Wall on NW edge |
| `WallNorth` | 0x2 | Wall on north edge |
| `WallNorthEast` | 0x4 | Wall on NE edge |
| `WallEast` | 0x8 | Wall on east edge |
| `WallSouthEast` | 0x10 | Wall on SE edge |
| `WallSouth` | 0x20 | Wall on south edge |
| `WallSouthWest` | 0x40 | Wall on SW edge |
| `WallWest` | 0x80 | Wall on west edge |
| `Loc` | 0x100 | Location (object) occupies tile |

**Tier 2 - Projectile blocker flags** (bits 9-17, 0x200-0x20000):
Shifted left by 9 bits from the movement equivalents. Used for line-of-sight checks. Example: `WallNorthProjBlocker` =
0x400, `LocProjBlocker` = 0x20000.

**Tier 3 - Route blocker flags** (bits 22-30, 0x400000-0x40000000):
Used by the `LINE_OF_SIGHT` collision strategy. These let pathfinding route *through* objects like bank booths while
still blocking projectiles. Example: `WallNorthRouteBlocker` = 0x800000, `locRouteBlocker` = 0x40000000.

**Special flags**:
| Flag | Hex | Purpose |
|------|-----|---------|
| `FloorDecoration` | 0x40000 | Floor decoration present |
| `Npc` | 0x80000 | NPC occupies tile (custom flag) |
| `Player` | 0x100000 | Player occupies tile (custom flag) |
| `Floor` | 0x200000 | Floor exists (used for blocked-strategy) |
| `Roof` | 0x80000000 | Roof present (used for indoors/outdoors strategies) |
| `Null` | 0x7FFFFFFF | Unallocated zone sentinel - blocks everything |

**Composite masks** (precomputed for the pathfinder):
| Mask | Hex | Composition |
|------|-----|-------------|
| `FloorBlocked` | 0x240000 | `Floor \| FloorDecoration` |
| `WalkBlocked` | 0x240100 | `FloorBlocked \| Loc` |
| `BlockWest` | 0x240108 | `WalkBlocked \| WallWest` |
| `BlockEast` | 0x240180 | `WalkBlocked \| WallEast` (note: inverted - checks the wall the entity would cross) |
| `BlockSouth` | 0x240102 | `WalkBlocked \| WallNorth` |
| `BlockNorth` | 0x240120 | `WalkBlocked \| WallSouth` |
| `BlockSouthWest` | 0x24010E | Corner composite |
| `BlockSouthEast` | 0x240183 | Corner composite |
| `BlockNorthWest` | 0x240138 | Corner composite |
| `BlockNorthEast` | 0x2401E0 | Corner composite |
| `BlockNorthAndSouthEast` | 0x24013E | Edge + corner composite (for size > 1) |
| `BlockNorthAndSouthWest` | 0x2401E3 | Edge + corner composite |
| `BlockNorthEastAndWest` | 0x24018F | Edge + corner composite |
| `BlockSouthEastAndWest` | 0x2401F8 | Edge + corner composite |

The `Block*RouteBlocker` variants mirror the above but use tier-3 route blocker flags.

### DirectionFlag (`flag/direction_flag.rs`)

`#[repr(u8)]` enum used by the BFS pathfinder to encode the direction from which each tile was reached (for backtracking
the path):

| Flag        | Value | Note            |
|-------------|-------|-----------------|
| `North`     | 0x1   |                 |
| `East`      | 0x2   |                 |
| `South`     | 0x4   |                 |
| `West`      | 0x8   |                 |
| `SouthWest` | 0xC   | `West \| South` |
| `NorthWest` | 0x9   | `West \| North` |
| `SouthEast` | 0x6   | `East \| South` |
| `NorthEast` | 0x3   | `East \| North` |

The naming convention is reversed from what you might expect: `DirectionFlag::East` (0x2) means "I arrived here *from*
the east" - i.e., the source is to the east, so move west to backtrack. This matches the direction you came FROM, not
the direction you're going.

### BlockAccessFlag (`flag/block_flag.rs`)

`#[repr(u8)]` enum for loc-specific approach restrictions. Some locs (e.g., a bank booth) can only be interacted with
from certain sides:

| Flag    | Value |
|---------|-------|
| `North` | 0x1   |
| `East`  | 0x2   |
| `South` | 0x4   |
| `West`  | 0x8   |

These are rotated by the loc's angle using `rotate_flags()`.

---

## Collision Strategies

### CollisionStrategy (`collision/collision_strategy.rs`)

A `CollisionStrategy` is a function pointer `fn(tile_flag: u32, block_flag: u32) -> bool` that determines whether a tile
is passable given its collision flags and the required blocking mask.

Five strategies exist:

**1. Normal** (`CollisionType::NORMAL`):

```
(tile_flag & block_flag) == Open
```

Standard movement. Returns true if none of the required blocking bits are set.

**2. Blocked** (`CollisionType::BLOCKED`):

```
(tile_flag & (block_flag & !Floor)) == Open && (tile_flag & Floor) != Open
```

Only allows movement onto tiles that HAVE a floor flag. Used for entities that can only walk on specific surfaces. The
floor flag is stripped from the blocking check but must be present.

**3. Indoors** (`CollisionType::INDOORS`):

```
(tile_flag & block_flag) == Open && (tile_flag & Roof) != Open
```

Normal collision check PLUS requires a roof flag. NPCs with this strategy stay inside buildings.

**4. Outdoors** (`CollisionType::OUTDOORS`):

```
(tile_flag & (block_flag | Roof)) == Open
```

Normal collision check PLUS the roof flag is treated as a blocker. Entities with this strategy cannot enter buildings.

**5. Line of Sight** (`CollisionType::LINE_OF_SIGHT`):

```
movement_flags = (block_flag & MOVEMENT) << 9
route_flags = (block_flag & ROUTE) >> 13
(tile_flag & (movement_flags | route_flags)) == Open
```

The most complex strategy. It shifts the movement-tier flags into the projectile-blocker range and the route-blocker
flags into a comparable range, then checks against the tile. This lets projectiles pass through normal walls but be
blocked by walls specifically flagged as projectile blockers, and lets pathing route through banker-style objects that
have route blockers.

---

## Pathfinder (BFS A*)

### PathFinder (`pathfinder.rs`)

The main pathfinding algorithm: a **Breadth-First Search** over a 128x128 local grid, producing an optimal shortest path
of up to 25 waypoints.

**Constants**:
| Constant | Value | Purpose |
|----------|-------|---------|
| `DEFAULT_SEARCH_MAP_SIZE` | 128 | Local search grid is 128x128 tiles |
| `SEARCH_HALF_MAP_SIZE` | 64 | Source is centered in the grid |
| `DEFAULT_RING_BUFFER_SIZE` | 4096 | BFS queue capacity (ring buffer) |
| `DEFAULT_DISTANCE_VALUE` | 99,999,999 | Unreached sentinel |
| `DEFAULT_SRC_DIRECTION_VALUE` | 99 | Source tile direction marker |
| `MAX_ALTERNATIVE_ROUTE_LOWEST_COST` | 1000 | Approach-point cost cap |
| `MAX_ALTERNATIVE_ROUTE_SEEK_RANGE` | 100 | Max BFS distance for approach |
| `MAX_ALTERNATIVE_ROUTE_DISTANCE_FROM_DESTINATION` | 10 | Search radius around dest for approach |

**Data structures** (all pre-allocated, reused across calls):

- `directions: Vec<i8>` (128*128) - BFS backtrack direction for each local tile
- `distances: Vec<i32>` (128*128) - BFS distance from source for each local tile
- `generations: Vec<u32>` (128*128) - Generation counter per tile (avoids full reset)
- `valid_local: Vec<u32>` (4096) - Ring buffer queue of packed (x, z) coordinates
- `waypoints: [u32; 25]` - Output buffer for path coordinates

**Generation-based reset**: Instead of zeroing the entire 16KB directions/distances arrays every call, the pathfinder
increments a `generation` counter. A tile is "unvisited" if its generation doesn't match. Full reset only happens on u32
wrap-around (~4.3 billion calls).

**Three size-specialized BFS functions**:

1. **`find_path_1`** - Source entity size 1x1. Simplest collision checks: each cardinal direction checks one tile, each
   diagonal checks three tiles (the diagonal tile + both adjacent cardinal tiles).

2. **`find_path_2`** - Source entity size 2x2. Checks the two edge tiles for each cardinal direction and three tiles for
   diagonals. Uses composite flags like `BlockNorthAndSouthEast`.

3. **`find_path_n`** - Source entity size NxN (3+). Checks corner tiles plus iterates the edge tiles between them.
   Generalizes the size-2 approach with loops.

**BFS expansion order**: For each tile, the algorithm tries 8 directions in order:

1. West (east-to-west)
2. East (west-to-east)
3. South (north-to-south)
4. North (south-to-north)
5. South-West (north-east to south-west)
6. South-East (north-west to south-east)
7. North-West (south-east to north-west)
8. North-East (south-west to north-east)

Each expansion checks:

1. Within bounds of the local 128x128 grid
2. Not already visited this generation
3. All relevant collision checks pass via the collision strategy function

**Path extraction**: After BFS completes (destination reached or queue exhausted), the path is extracted by backtracking
from the destination using the `directions` array. Each time the direction changes, a new waypoint is recorded. The
waypoints are then reversed (they were recorded dest-to-src).

**Move-near / approach point**: If the exact destination is unreachable and `move_near` is true,
`find_closest_approach_point` searches a 21x21 area around the destination (+-10 tiles) for the reachable tile with
minimum squared distance to the destination rectangle. Ties are broken by preferring shorter BFS distance. Width/height
are rotated by the loc's angle.

**Ring buffer BFS queue**: The queue uses a power-of-2 ring buffer (size 4096) with reader/writer indices masked by
`& 4095`. This avoids VecDeque heap allocation.

---

## Naive Pathfinder

### find_naive_path (`naive_pathfinder.rs`)

A simplified greedy pathfinder used for NPC movement. Instead of full BFS, it walks step-by-step toward the destination
using the step validator.

**Algorithm**:

1. If source and destination rectangles **intersect**, pick a random cardinal direction to move out of the way.
2. Compute the **naive destination** - the tile on the source entity's perimeter closest to the destination's south-west
   corner, calculated using diagonal/anti-diagonal bisection of the 2D plane.
3. If that destination is **diagonal** to the target (touching only at a corner), return it immediately (can't interact
   diagonally).
4. If the destination **intersects** the target, return it (already at interaction range).
5. Otherwise, **walk greedily**: each step tries diagonal movement first, then cardinal X, then cardinal Z. Stops when
   blocked or when aligned on both axes.

**Naive destination calculation** (`naive_destination`): The function determines which cardinal side of the target the
source lies on by computing two discriminants:

- `diagonal = srcX - destX + (srcZ - destZ)` (the `\` diagonal)
- `anti = srcX - destX - (srcZ - destZ)` (the `/` anti-diagonal)

Four boolean tests divide the plane into West/North/East/South quadrants. Within each quadrant, the offset along the
target's side is computed by clamping the diagonal/anti-diagonal values to the target's dimensions.

Returns `None` if the source is exactly on a corner (no clear cardinal side), which causes the pathfinder to return an
empty path.

**Static result buffer**: Uses a `static mut RESULT: [u32; 1]` since the naive pathfinder always returns exactly 0 or 1
waypoints.

---

## Line of Sight / Line of Walk

### Line Validator (`line_validator.rs`)

Boolean line-of-sight and line-of-walk checks. Returns `true`/`false` without producing a path.

**`has_line_of_sight`**: Uses projectile-blocker flags (`LocProjBlocker`, `Wall*ProjBlocker`). Checks whether a
projectile can reach the target without being blocked. On the source tile, the full `Loc` flag is checked (standing on a
loc blocks LoS). At the destination tile, the `ProjBlocker` component of the directional flag is stripped (you can shoot
AT an object but not THROUGH it).

**`has_line_of_walk`**: Uses movement flags (`WallNorth`, `WalkBlocked`, etc). Checks whether a straight-line walk is
possible.

Both delegate to `ray_cast_line` which implements a Bresenham-style ray cast:

**Ray cast algorithm**:

1. Compute start/end points by clamping source and destination to each other's rectangles using `Line::coordinate`.
2. If start == end, return true (same tile or overlapping).
3. For LoS: if the start tile itself has `flag_loc` set, return false (standing on a blocking object).
4. Determine the major axis (whichever has larger absolute delta).
5. Step along the major axis one tile at a time. For each step:
    - Check the tile in the major direction against `x_flags` or `z_flags`.
    - Use fixed-point scaled coordinates (16-bit fractional part) to track the minor axis.
    - When the minor axis crosses a tile boundary, also check the new tile against the minor direction flag.
6. At the destination tile (for LoS only), the `flag_proj` component is stripped from the directional check flags,
   allowing the ray to reach the destination object.

**Fixed-point math** (`line.rs`):

- `HALF_TILE` = 32768 (1 << 15, which is half of 1 << 16)
- `scale_up(tiles)` = tiles << 16 (convert to fixed-point)
- `scale_down(tiles)` = tiles >> 16 (convert back)
- The tangent is `delta_minor << 16 / abs_delta_major`, giving sub-tile precision for the minor axis.

**`Line::coordinate`**: Clamps coordinate `b` to the range `[a, a + size - 1]`. Used to find the nearest point on one
rectangle's edge to the other rectangle. This ensures the ray cast starts/ends at the closest tile pair between two
potentially multi-tile entities.

---

## Line Pathfinder (Ray Cast Path)

### line_pathfinder.rs

Same ray-cast algorithm as `line_validator.rs`, but instead of returning a bool, it records every tile coordinate along
the ray into a static buffer and returns the path as a slice.

**`line_of_sight`**: Returns coordinates along the LoS ray (using projectile-blocker flags). Returns empty slice if
blocked.

**`line_of_walk`**: Returns coordinates along the walk ray (using movement flags). Returns empty slice if blocked.

**Static buffer**: `LINE_BUFFER: [u32; 128]` limits the maximum ray length to 128 tiles. Each entry is a packed
`CoordGrid`.

Both functions can produce multiple coordinates per major-axis step when the minor axis crosses a tile boundary (the
diagonal tile is included as a separate coordinate in the path).

---

## Step Validator

### can_travel (`step_validator.rs`)

Validates a single-step movement in any of 8 directions for entities of any size.

`can_travel(flags, y, x, z, offset_x, offset_z, size, extra_flag, collision)` dispatches on the `(offset_x, offset_z)`
pair to one of 8 directional check functions.

**Directional checks by entity size**:

For **size 1**: Each cardinal direction checks 1 tile. Each diagonal checks 3 tiles (the diagonal tile + both adjacent
cardinals). For example, moving southwest checks:

- `(x-1, z-1)` with `BlockSouthWest`
- `(x-1, z)` with `BlockWest`
- `(x, z-1)` with `BlockSouth`

For **size 2**: Each cardinal direction checks 2 tiles (the two edge tiles that enter the new column/row). Each diagonal
checks 3 tiles using composite flags.

For **size N** (3+): Each cardinal direction checks corner tiles with corner flags, plus iterates the interior edge
tiles with edge-composite flags (e.g., `BlockNorthAndSouthEast` for tiles along the west edge).

**Extra flag**: An additional flag ORed into every blocking check. Used to make NPC/player collision optional - pass
`CollisionFlag::Npc as u32` to block movement through NPCs.

---

## Reach Strategy

### ReachStrategy (`reach/reach_strategy.rs`)

Determines whether a source entity is "in range" to interact with a destination object, based on the object's shape.

**Exit strategies** (determined by `shape`):
| Shape value | Strategy | Used for |
|-------------|----------|----------|
| -2 | `RECTANGLE_EXCLUSIVE` | Rectangle reach, must NOT overlap |
| -1 | `NO_STRATEGY` | No shape-based reach (always false) |
| 0-3, 9 | `WALL` | Wall objects |
| 4-8 | `WALL_DECOR` | Wall decoration objects |
| 10-11, 22 | `RECTANGLE` | Standard loc/ground decoration |

**Quick exit**: If strategy is not `RECTANGLE_EXCLUSIVE` and `src == dest`, return true immediately (standing on the
destination).

### Wall reach (`reach_wall_1`, `reach_wall_n`)

For wall objects, reach is determined by adjacency + direction. The source must be on the correct side of the wall and
the intervening wall flag must not be set.

For `WallStraight` (shape 0): The entity must be on the opposite side of the wall. Adjacent tiles on the perpendicular
axis also count if no blocking wall exists between them.

For `WallL` (shape 2): Two sides are directly reachable (the two sides the L-wall covers). The other two sides require
no wall between.

For `WallDiagonal` (shape 9): The entity must be adjacent on any cardinal side, with the appropriate wall flag clear.

Size-N variants extend these checks to handle multi-tile sources, checking range overlap instead of exact adjacency.

### Wall decor reach (`reach_wall_decor_1`, `reach_wall_decor_n`)

For wall decorations (shapes 4-8). Shape 7 gets a special +2 rotation via `altered_rotation`. Shapes 6 and 7 use
two-sided checks (two adjacent tiles). Shape 8 uses four-sided checks (all cardinal neighbors).

### Rectangle reach (`reach_rectangle`, `reach_exclusive_rectangle`)

For standard locs and ground decorations. Handles rotation of width/height and block access flags.

**`reach_rectangle`**: Source has reached the destination if it either **overlaps** the destination OR is **adjacent**
to it (via `reach_rectangle_1` or `reach_rectangle_n`).

**`reach_exclusive_rectangle`**: Source has reached the destination ONLY if it is **adjacent** and does NOT overlap.
Used for shape -2 (e.g., interacting with objects you shouldn't stand on).

---

## Rectangle Boundary

### rectangle_boundary.rs

Low-level adjacency checks for rectangle-to-rectangle reach.

**`collides`**: AABB intersection test. Returns true if two rectangles overlap at all.

**`reach_rectangle_1`** (source size 1): Checks if the 1x1 source is cardinally adjacent to the destination rectangle
and no wall flag blocks approach from that direction. Also respects `block_access_flags` - if approach from a given side
is blocked by the loc's configuration, that side returns false.

Checks in order:

1. West side: `src_x == dest_x - 1` in z-range, no `WallEast` on source, west access not blocked
2. East side: `src_x == east + 1` in z-range, no `WallWest` on source, east access not blocked
3. South side: `src_z + 1 == dest_z` in x-range, no `WallNorth` on source, south access not blocked
4. North side: `src_z == north + 1` in x-range, no `WallSouth` on source, north access not blocked

**`reach_rectangle_n`** (source size N): Checks if ANY tile along the overlapping edge between the two rectangles has no
wall blocking. Iterates the overlapping range on each cardinal side.

---

## Wall Collision Modification

### lib.rs: `change_wall`, `change_wall_straight`, `change_wall_corner`, `change_wall_l`

When a wall is placed or removed, collision flags must be set on BOTH tiles the wall separates (the wall exists on the
boundary between two tiles).

**Three-tier recursive application**: When a wall has `breakroutefinding=true`, it recursively calls itself with
`breakroutefinding=false` (to also apply the regular tier). When `blockrange=true`, it recursively calls itself with
`blockrange=false`. This means a single wall placement with both flags applies collision in three layers:

1. Route blocker flags (tier 3)
2. Projectile blocker flags (tier 2)
3. Movement flags (tier 1)

**Wall types by shape**:

`WallStraight` (shape 0): Sets flags on the tile and the tile on the opposite side.

- Angle 0 (West): Sets `WallWest` on `(x,z)` and `WallEast` on `(x-1,z)`
- Angle 1 (North): Sets `WallNorth` on `(x,z)` and `WallSouth` on `(x,z+1)`
- Angle 2 (East): Sets `WallEast` on `(x,z)` and `WallWest` on `(x+1,z)`
- Angle 3 (South): Sets `WallSouth` on `(x,z)` and `WallNorth` on `(x,z-1)`

`WallCorner` (shapes 1, 3): Sets diagonal wall flags.

- Angle 0 (West): `WallNorthWest` on `(x,z)` and `WallSouthEast` on `(x-1,z+1)`
- Angle 1 (North): `WallNorthEast` on `(x,z)` and `WallSouthWest` on `(x+1,z+1)`
- Angle 2 (East): `WallSouthEast` on `(x,z)` and `WallNorthWest` on `(x+1,z-1)`
- Angle 3 (South): `WallSouthWest` on `(x,z)` and `WallNorthEast` on `(x-1,z-1)`

`WallL` (shape 2): Sets TWO wall directions on the main tile plus flags on TWO adjacent tiles.

- Angle 0 (West): `WallNorth|WallWest` on `(x,z)`, `WallEast` on `(x-1,z)`, `WallSouth` on `(x,z+1)`
- Angle 1 (North): `WallNorth|WallEast` on `(x,z)`, `WallSouth` on `(x,z+1)`, `WallWest` on `(x+1,z)`
- Angle 2 (East): `WallSouth|WallEast` on `(x,z)`, `WallWest` on `(x+1,z)`, `WallNorth` on `(x,z-1)`
- Angle 3 (South): `WallSouth|WallWest` on `(x,z)`, `WallNorth` on `(x,z-1)`, `WallEast` on `(x-1,z)`

### Other collision modifiers

**`change_loc`**: Sets/removes `Loc` (+ optionally `LocProjBlocker` and `locRouteBlocker`) on all tiles in the loc's
width*length footprint.

**`change_npc`** / **`change_player`**: Sets/removes `Npc` / `Player` flags on all tiles in the entity's size*size
footprint.

**`change_floor`**: Sets/removes the `Floor` flag on a single tile.

**`change_roof`**: Sets/removes the `Roof` flag on a single tile.

---

## Location Types

### LocAngle (`loc_angle.rs`)

Four rotations for locs:
| Value | Name | Direction |
|-------|------|-----------|
| 0 | West | Default orientation |
| 1 | North | 90 degrees clockwise |
| 2 | East | 180 degrees |
| 3 | South | 270 degrees clockwise |

Invalid values cause `process::abort()`.

### LocShape (`loc_shape.rs`)

23 distinct loc shapes:
| Value | Name | Category |
|-------|------|----------|
| 0 | WallStraight | Wall |
| 1 | WallDiagonalCorner | Wall |
| 2 | WallL | Wall |
| 3 | WallSquareCorner | Wall |
| 4-8 | WallDecor* | Wall decoration |
| 9 | WallDiagonal | Wall (diagonal) |
| 10 | CentrepieceStraight | Ground object |
| 11 | CentrepieceDiagonal | Ground object |
| 12-21 | Roof* | Roof variants |
| 22 | GroundDecor | Ground decoration |

### LocLayer (`loc_layer.rs`)

Four rendering layers:
| Value | Name |
|-------|------|
| 0 | Wall |
| 1 | WallDecor |
| 2 | Ground |
| 3 | GroundDecor |

---

## Rotation Utilities

### rotation.rs

**`rotate(angle, a, b) -> u8`**: Returns `a` when angle is even (0, 2) and `b` when angle is odd (1, 3). Used to swap
width/height for rotated locs. Implementation uses a bitmask trick: `mask = (angle & 1).wrapping_neg()` produces 0x00 or
0xFF, then `(a & !mask) | (b & mask)` selects branchlessly.

**`rotate_flags(angle, block_access_flags) -> u8`**: Rotates the 4-bit block access flags clockwise by `angle`
positions. Uses circular bit rotation: `((flags << angle) & 0xf) | (flags >> (4 - angle))`.

---

## Global State and Safety Model

### lib.rs globals

The crate uses two `static mut` globals wrapped in `Lazy`:

```rust
static mut COLLISION_FLAGS: Lazy<CollisionFlagMap> = Lazy::new(CollisionFlagMap::new);
static mut PATHFINDER: Lazy<PathFinder> = Lazy::new(PathFinder::new);
```

**Safety invariant**: All writes to `COLLISION_FLAGS` happen on the single-threaded engine tick (loc/npc/player
add/remove). Reads happen from both the tick thread and async pathfinding. During async phases, no writer runs, so
concurrent reads are sound.

`PATHFINDER` is only used from the tick thread (sync pathfinding). Async pathfinding would need its own `PathFinder`
instances.

The crate suppresses safety-related lints:

```rust
#![allow(static_mut_refs)]
#![allow(unsafe_op_in_unsafe_fn)]
```

All pointer arithmetic (`as_ptr().add(...)`, `as_mut_ptr().add(...)`) bypasses bounds checks for performance. This is
safe as long as indices stay within allocated bounds (guaranteed by the coordinate masking and search grid size
constraints).

---

## Public API

All public functions in `lib.rs` are thin wrappers that convert external types (`u8`, `u16`) to internal `i32` and
delegate to the internal modules through the global `COLLISION_FLAGS` and `PATHFINDER`:

| Function                     | Returns          | Purpose                                 |
|------------------------------|------------------|-----------------------------------------|
| `find_path(...)`             | `&'static [u32]` | BFS pathfind, sync tick-thread only     |
| `find_naive_path(...)`       | `&'static [u32]` | Greedy NPC pathfind                     |
| `has_line_of_sight(...)`     | `bool`           | Projectile LoS check                    |
| `has_line_of_walk(...)`      | `bool`           | Walk LoS check                          |
| `line_of_sight(...)`         | `&'static [u32]` | LoS ray with coordinates                |
| `line_of_walk(...)`          | `&'static [u32]` | Walk ray with coordinates               |
| `reached(...)`               | `bool`           | Interaction range check                 |
| `can_travel(...)`            | `bool`           | Single-step movement check              |
| `change_floor(...)`          | `()`             | Add/remove floor flag                   |
| `change_loc(...)`            | `()`             | Add/remove loc collision                |
| `change_npc(...)`            | `()`             | Add/remove NPC collision                |
| `change_player(...)`         | `()`             | Add/remove player collision             |
| `change_roof(...)`           | `()`             | Add/remove roof flag                    |
| `change_wall(...)`           | `()`             | Add/remove wall collision (all 3 tiers) |
| `allocate_if_absent(...)`    | `()`             | Pre-allocate a zone                     |
| `deallocate_if_present(...)` | `()`             | Free a zone                             |
| `is_zone_allocated(...)`     | `bool`           | Check zone allocation                   |
| `is_flagged(...)`            | `bool`           | Check specific flags on a tile          |
| `__set(...)`                 | `()`             | Direct flag set (used by benchmark)     |

All path-returning functions return `&'static [u32]` slices backed by internal static buffers. This means they are only
valid until the next call to the same function.

---

## Benchmark Harness

### main.rs

A benchmark binary that:

1. Loads `lumbridge.json` - a pre-exported collision flag map for the Lumbridge map square (3200-3263, 3200-3263).
2. Applies the flags using `__set` with the packed zone indexing:
   `lumbridge[((z & 0x3f) | ((x & 0x3f) << 6))][((x & 0x7) | ((z & 0x7) << 3))]`.
3. Runs 100,000 pathfinding calls in a loop (3232,3205 -> 3232,3215, size 1, normal collision) and prints timing.
4. Sleeps 600ms between batches (matching the game tick rate).

---

## 💪 Benchmarks

I have created `benchmark.rs` for synthetic examples of performance.
They load in a full reconstruction of the Lumbridge mapsquare (64x64) tiles
with full clipping flags set. Then in a loop it runs through 100k pathfinder
requests to a destination +10 tiles North with a single access point to the destination.

## Ran in Release mode on Windows x64 OS.

```
100k paths took: 133.4965ms; time per call: 1.334µs
100k paths took: 134.3726ms; time per call: 1.343µs
100k paths took: 132.8235ms; time per call: 1.328µs
100k paths took: 132.8823ms; time per call: 1.328µs
100k paths took: 134.2985ms; time per call: 1.342µs
100k paths took: 133.489ms; time per call: 1.334µs
100k paths took: 133.2397ms; time per call: 1.332µs
100k paths took: 133.3598ms; time per call: 1.333µs
100k paths took: 133.1242ms; time per call: 1.331µs
```

## Dependencies

| Crate        | Version | Purpose                                                                             |
|--------------|---------|-------------------------------------------------------------------------------------|
| `once_cell`  | 1.21    | Lazy initialization of global collision map and pathfinder state                    |
| `rand`       | 0.10    | Random direction selection in the naive pathfinder when source overlaps destination |
| `serde_json` | 1.0     | Deserializing collision flag maps from JSON (benchmark harness)                     |
