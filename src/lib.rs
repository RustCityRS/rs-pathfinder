#![allow(static_mut_refs)]
#![allow(unsafe_op_in_unsafe_fn)]

use once_cell::sync::Lazy;

use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::*;
use crate::rsmod::collision::zone::ZoneFlagMap;
use crate::rsmod::flag::collision_flag::CollisionFlag;
use crate::rsmod::loc_angle::LocAngle;
use crate::rsmod::loc_shape::LocShape;
use crate::rsmod::pathfinder::PathFinder;
use crate::rsmod::reach::reach_strategy::ReachStrategy;

pub mod rsmod;

// `COLLISION_FLAGS` is a global mutable map. Writes happen from the
// single-threaded engine tick (loc/npc/player add/remove). Reads happen
// from both sync and async (pooled) pathfinding. During any async phase,
// no writer runs — so concurrent reads are sound.
static mut COLLISION_FLAGS: Lazy<CollisionFlagMap> = Lazy::new(CollisionFlagMap::new);

// `ZONE_FLAGS` is a zone-granularity byte plane for area attributes
// (f2p / multiway) sharing `COLLISION_FLAGS`'s zone indexing. Same safety
// story: writes only from the single-threaded tick (seeded at startup,
// rare runtime toggles), reads from tick and async pathfinding while no
// writer runs.
static mut ZONE_FLAGS: Lazy<ZoneFlagMap> = Lazy::new(ZoneFlagMap::new);

// Single PathFinder for synchronous tick-thread callers (player
// movement, script ops, interactive clicks).
// "sync stage uses the global instance" pattern — this is the one
// instance reused across sequential calls during the tick.
static mut PATHFINDER: Lazy<PathFinder> = Lazy::new(PathFinder::new);

/// **Sync** pathfind — uses the single tick-thread PathFinder. Call
/// this from anywhere during the single-threaded tick (player walk,
/// script `path` ops, click-to-move, etc.).
///
/// # Safety
/// Must only be called from the tick thread while no async pathfind
/// phase is in progress.
#[allow(clippy::too_many_arguments)]
pub fn find_path(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_size: u8,
    dest_width: u8,
    dest_length: u8,
    angle: u8,
    shape: i8,
    move_near: bool,
    block_access_flags: u8,
    max_waypoints: u8,
    collision: CollisionType,
) -> &'static [u32] {
    unsafe {
        with_collision_strategy!(collision, strategy => PATHFINDER.find_path(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_size,
            dest_width,
            dest_length,
            angle,
            shape,
            move_near,
            block_access_flags,
            max_waypoints,
            strategy,
        ))
    }
}

#[allow(clippy::too_many_arguments)]
pub fn find_naive_path(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_width: u8,
    src_length: u8,
    dest_width: u8,
    dest_length: u8,
    extra_flag: u32,
    collision: CollisionType,
) -> &'static [u32] {
    unsafe {
        with_collision_strategy!(collision, strategy => rsmod::naive_pathfinder::find_naive_path(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_width,
            src_length,
            dest_width,
            dest_length,
            extra_flag,
            strategy,
        ))
    }
}

pub fn change_floor(x: u16, z: u16, y: u8, add: bool) {
    unsafe {
        if add {
            COLLISION_FLAGS.add(x as i32, z as i32, y as i32, CollisionFlag::Floor as u32);
        } else {
            COLLISION_FLAGS.remove(x as i32, z as i32, y as i32, CollisionFlag::Floor as u32);
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn change_loc(
    x: u16,
    z: u16,
    y: u8,
    width: u8,
    length: u8,
    blockrange: bool,
    breakroutefinding: bool,
    add: bool,
) {
    let mut mask: u32 = CollisionFlag::Loc as u32;
    if blockrange {
        mask |= CollisionFlag::LocProjBlocker as u32;
    }
    if breakroutefinding {
        mask |= CollisionFlag::LocRouteBlocker as u32;
    }
    let x = x as i32;
    let z = z as i32;
    let y = y as i32;
    let width = width as i32;
    let length = length as i32;
    let area = width * length;
    unsafe {
        if add {
            for index in 0..area {
                COLLISION_FLAGS.add(x + (index % width), z + (index / width), y, mask);
            }
        } else {
            for index in 0..area {
                COLLISION_FLAGS.remove(x + (index % width), z + (index / width), y, mask);
            }
        }
    }
}

pub fn change_npc(x: u16, z: u16, y: u8, size: u8, add: bool) {
    change_entity(x, z, y, size, add, CollisionFlag::Npc as u32);
}

pub fn change_player(x: u16, z: u16, y: u8, size: u8, add: bool) {
    change_entity(x, z, y, size, add, CollisionFlag::Player as u32);
}

#[inline(always)]
fn change_entity(x: u16, z: u16, y: u8, size: u8, add: bool, mask: u32) {
    let x = x as i32;
    let z = z as i32;
    let y = y as i32;
    unsafe {
        if size == 1 {
            if add {
                COLLISION_FLAGS.add(x, z, y, mask);
            } else {
                COLLISION_FLAGS.remove(x, z, y, mask);
            }
            return;
        }
        let size = size as i32;
        for dz in 0..size {
            for dx in 0..size {
                if add {
                    COLLISION_FLAGS.add(x + dx, z + dz, y, mask);
                } else {
                    COLLISION_FLAGS.remove(x + dx, z + dz, y, mask);
                }
            }
        }
    }
}

pub fn change_roof(x: u16, z: u16, y: u8, add: bool) {
    unsafe {
        if add {
            COLLISION_FLAGS.add(x as i32, z as i32, y as i32, CollisionFlag::Roof as u32);
        } else {
            COLLISION_FLAGS.remove(x as i32, z as i32, y as i32, CollisionFlag::Roof as u32);
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn change_wall(
    x: u16,
    z: u16,
    y: u8,
    angle: u8,
    shape: i8,
    blockrange: bool,
    breakroutefinding: bool,
    add: bool,
) {
    let x = x as i32;
    let z = z as i32;
    let y = y as i32;
    unsafe {
        match LocShape::from(shape) {
            LocShape::WallStraight => {
                change_wall_straight(x, z, y, angle, blockrange, breakroutefinding, add)
            }
            LocShape::WallDiagonalCorner | LocShape::WallSquareCorner => {
                change_wall_corner(x, z, y, angle, blockrange, breakroutefinding, add)
            }
            LocShape::WallL => change_wall_l(x, z, y, angle, blockrange, breakroutefinding, add),
            _ => {}
        }
    }
}

#[inline(always)]
unsafe fn change_wall_straight(
    x: i32,
    z: i32,
    y: i32,
    angle: u8,
    blockrange: bool,
    breakroutefinding: bool,
    add: bool,
) {
    let west: u32 = if breakroutefinding {
        CollisionFlag::WallWestRouteBlocker
    } else if blockrange {
        CollisionFlag::WallWestProjBlocker
    } else {
        CollisionFlag::WallWest
    } as u32;
    let east: u32 = if breakroutefinding {
        CollisionFlag::WallEastRouteBlocker
    } else if blockrange {
        CollisionFlag::WallEastProjBlocker
    } else {
        CollisionFlag::WallEast
    } as u32;
    let north: u32 = if breakroutefinding {
        CollisionFlag::WallNorthRouteBlocker
    } else if blockrange {
        CollisionFlag::WallNorthProjBlocker
    } else {
        CollisionFlag::WallNorth
    } as u32;
    let south: u32 = if breakroutefinding {
        CollisionFlag::WallSouthRouteBlocker
    } else if blockrange {
        CollisionFlag::WallSouthProjBlocker
    } else {
        CollisionFlag::WallSouth
    } as u32;

    match LocAngle::from(angle) {
        LocAngle::West => {
            if add {
                COLLISION_FLAGS.add(x, z, y, west);
                COLLISION_FLAGS.add(x - 1, z, y, east);
            } else {
                COLLISION_FLAGS.remove(x, z, y, west);
                COLLISION_FLAGS.remove(x - 1, z, y, east);
            }
        }
        LocAngle::North => {
            if add {
                COLLISION_FLAGS.add(x, z, y, north);
                COLLISION_FLAGS.add(x, z + 1, y, south);
            } else {
                COLLISION_FLAGS.remove(x, z, y, north);
                COLLISION_FLAGS.remove(x, z + 1, y, south);
            }
        }
        LocAngle::East => {
            if add {
                COLLISION_FLAGS.add(x, z, y, east);
                COLLISION_FLAGS.add(x + 1, z, y, west);
            } else {
                COLLISION_FLAGS.remove(x, z, y, east);
                COLLISION_FLAGS.remove(x + 1, z, y, west);
            }
        }
        LocAngle::South => {
            if add {
                COLLISION_FLAGS.add(x, z, y, south);
                COLLISION_FLAGS.add(x, z - 1, y, north);
            } else {
                COLLISION_FLAGS.remove(x, z, y, south);
                COLLISION_FLAGS.remove(x, z - 1, y, north);
            }
        }
    }
    if breakroutefinding {
        return change_wall_straight(x, z, y, angle, blockrange, false, add);
    }
    if blockrange {
        change_wall_straight(x, z, y, angle, false, false, add)
    }
}

#[inline(always)]
unsafe fn change_wall_corner(
    x: i32,
    z: i32,
    y: i32,
    angle: u8,
    blockrange: bool,
    breakroutefinding: bool,
    add: bool,
) {
    let north_west: u32 = if breakroutefinding {
        CollisionFlag::WallNorthWestRouteBlocker
    } else if blockrange {
        CollisionFlag::WallNorthWestProjBlocker
    } else {
        CollisionFlag::WallNorthWest
    } as u32;
    let south_east: u32 = if breakroutefinding {
        CollisionFlag::WallSouthEastRouteBlocker
    } else if blockrange {
        CollisionFlag::WallSouthEastProjBlocker
    } else {
        CollisionFlag::WallSouthEast
    } as u32;
    let north_east: u32 = if breakroutefinding {
        CollisionFlag::WallNorthEastRouteBlocker
    } else if blockrange {
        CollisionFlag::WallNorthEastProjBlocker
    } else {
        CollisionFlag::WallNorthEast
    } as u32;
    let south_west: u32 = if breakroutefinding {
        CollisionFlag::WallSouthWestRouteBlocker
    } else if blockrange {
        CollisionFlag::WallSouthWestProjBlocker
    } else {
        CollisionFlag::WallSouthWest
    } as u32;

    match LocAngle::from(angle) {
        LocAngle::West => {
            if add {
                COLLISION_FLAGS.add(x, z, y, north_west);
                COLLISION_FLAGS.add(x - 1, z + 1, y, south_east);
            } else {
                COLLISION_FLAGS.remove(x, z, y, north_west);
                COLLISION_FLAGS.remove(x - 1, z + 1, y, south_east);
            }
        }
        LocAngle::North => {
            if add {
                COLLISION_FLAGS.add(x, z, y, north_east);
                COLLISION_FLAGS.add(x + 1, z + 1, y, south_west);
            } else {
                COLLISION_FLAGS.remove(x, z, y, north_east);
                COLLISION_FLAGS.remove(x + 1, z + 1, y, south_west);
            }
        }
        LocAngle::East => {
            if add {
                COLLISION_FLAGS.add(x, z, y, south_east);
                COLLISION_FLAGS.add(x + 1, z - 1, y, north_west);
            } else {
                COLLISION_FLAGS.remove(x, z, y, south_east);
                COLLISION_FLAGS.remove(x + 1, z - 1, y, north_west);
            }
        }
        LocAngle::South => {
            if add {
                COLLISION_FLAGS.add(x, z, y, south_west);
                COLLISION_FLAGS.add(x - 1, z - 1, y, north_east);
            } else {
                COLLISION_FLAGS.remove(x, z, y, south_west);
                COLLISION_FLAGS.remove(x - 1, z - 1, y, north_east);
            }
        }
    }
    if breakroutefinding {
        return change_wall_corner(x, z, y, angle, blockrange, false, add);
    }
    if blockrange {
        change_wall_corner(x, z, y, angle, false, false, add)
    }
}

#[inline(always)]
unsafe fn change_wall_l(
    x: i32,
    z: i32,
    y: i32,
    angle: u8,
    blockrange: bool,
    breakroutefinding: bool,
    add: bool,
) {
    let west: u32 = if breakroutefinding {
        CollisionFlag::WallWestRouteBlocker
    } else if blockrange {
        CollisionFlag::WallWestProjBlocker
    } else {
        CollisionFlag::WallWest
    } as u32;
    let east: u32 = if breakroutefinding {
        CollisionFlag::WallEastRouteBlocker
    } else if blockrange {
        CollisionFlag::WallEastProjBlocker
    } else {
        CollisionFlag::WallEast
    } as u32;
    let north: u32 = if breakroutefinding {
        CollisionFlag::WallNorthRouteBlocker
    } else if blockrange {
        CollisionFlag::WallNorthProjBlocker
    } else {
        CollisionFlag::WallNorth
    } as u32;
    let south: u32 = if breakroutefinding {
        CollisionFlag::WallSouthRouteBlocker
    } else if blockrange {
        CollisionFlag::WallSouthProjBlocker
    } else {
        CollisionFlag::WallSouth
    } as u32;

    match LocAngle::from(angle) {
        LocAngle::West => {
            if add {
                COLLISION_FLAGS.add(x, z, y, north | west);
                COLLISION_FLAGS.add(x - 1, z, y, east);
                COLLISION_FLAGS.add(x, z + 1, y, south);
            } else {
                COLLISION_FLAGS.remove(x, z, y, north | west);
                COLLISION_FLAGS.remove(x - 1, z, y, east);
                COLLISION_FLAGS.remove(x, z + 1, y, south);
            }
        }
        LocAngle::North => {
            if add {
                COLLISION_FLAGS.add(x, z, y, north | east);
                COLLISION_FLAGS.add(x, z + 1, y, south);
                COLLISION_FLAGS.add(x + 1, z, y, west);
            } else {
                COLLISION_FLAGS.remove(x, z, y, north | east);
                COLLISION_FLAGS.remove(x, z + 1, y, south);
                COLLISION_FLAGS.remove(x + 1, z, y, west);
            }
        }
        LocAngle::East => {
            if add {
                COLLISION_FLAGS.add(x, z, y, south | east);
                COLLISION_FLAGS.add(x + 1, z, y, west);
                COLLISION_FLAGS.add(x, z - 1, y, north);
            } else {
                COLLISION_FLAGS.remove(x, z, y, south | east);
                COLLISION_FLAGS.remove(x + 1, z, y, west);
                COLLISION_FLAGS.remove(x, z - 1, y, north);
            }
        }
        LocAngle::South => {
            if add {
                COLLISION_FLAGS.add(x, z, y, south | west);
                COLLISION_FLAGS.add(x, z - 1, y, north);
                COLLISION_FLAGS.add(x - 1, z, y, east);
            } else {
                COLLISION_FLAGS.remove(x, z, y, south | west);
                COLLISION_FLAGS.remove(x, z - 1, y, north);
                COLLISION_FLAGS.remove(x - 1, z, y, east);
            }
        }
    }
    if breakroutefinding {
        return change_wall_l(x, z, y, angle, blockrange, false, add);
    }
    if blockrange {
        change_wall_l(x, z, y, angle, false, false, add)
    }
}

pub fn allocate_if_absent(x: u16, z: u16, y: u8) {
    unsafe {
        COLLISION_FLAGS.allocate_if_absent(x as i32, z as i32, y as i32);
    }
}

pub fn deallocate_if_present(x: u16, z: u16, y: u8) {
    unsafe {
        COLLISION_FLAGS.deallocate_if_present(x as i32, z as i32, y as i32);
    }
}

pub fn is_zone_allocated(x: u16, z: u16, y: u8) -> bool {
    unsafe { COLLISION_FLAGS.is_zone_allocated(x as i32, z as i32, y as i32) }
}

pub fn is_flagged(x: u16, z: u16, y: u8, masks: u32) -> bool {
    unsafe { COLLISION_FLAGS.is_flagged(x as i32, z as i32, y as i32, masks) }
}

/* Prefetch hints for callers walking many entities in one tick (npc movement). Each collision
 * lookup is two dependent loads (zone table entry, then the zone's tiles) that usually miss
 * cache, so call `prefetch_zone` for an entity several ahead in the loop and `prefetch_tile`
 * for one a few ahead; by the time that entity steps, both loads are cached. Pure hints: they
 * never change any result. */
pub fn prefetch_zone(x: u16, z: u16, y: u8) {
    unsafe { COLLISION_FLAGS.prefetch_zone(x as i32, z as i32, y as i32) }
}

pub fn prefetch_tile(x: u16, z: u16, y: u8) {
    unsafe { COLLISION_FLAGS.prefetch_tile(x as i32, z as i32, y as i32) }
}

/* Flags the whole 8x8 zone containing (x, z). */
pub fn change_zone(x: u16, z: u16, y: u8, mask: u8, add: bool) {
    unsafe {
        ZONE_FLAGS.change_zone(x as i32, z as i32, y as i32, mask, add);
    }
}

pub fn is_zone_flagged(x: u16, z: u16, y: u8, mask: u8) -> bool {
    unsafe { ZONE_FLAGS.is_flagged(x as i32, z as i32, y as i32, mask) }
}

/* True if any orthogonally-adjacent tile lies in a zone carrying `mask`.
 * Used at map load so collision extends one tile into members land
 * bordering f2p; clamps at x=0 / z=0. */
pub fn borders_zone_flag(x: u16, z: u16, y: u8, mask: u8) -> bool {
    unsafe { ZONE_FLAGS.borders(x as i32, z as i32, y as i32, mask) }
}

#[allow(clippy::too_many_arguments)]
pub fn can_travel(
    y: u8,
    x: u16,
    z: u16,
    offset_x: i8,
    offset_z: i8,
    size: u8,
    extra_flag: u32,
    collision: CollisionType,
) -> bool {
    unsafe {
        with_collision_strategy!(collision, strategy => rsmod::step_validator::can_travel(
            &COLLISION_FLAGS,
            y as i32,
            x as i32,
            z as i32,
            offset_x,
            offset_z,
            size,
            extra_flag,
            strategy,
        ))
    }
}

#[inline]
pub fn can_travel_fast(y: u8, x: u16, z: u16, dx: i8, dz: i8, extra: u32) -> i8 {
    let y = y as i32;
    let x = x as i32;
    let z = z as i32;
    let x_dir = if dx < 0 { 3 } else { 4 };
    let z_dir = if dz < 0 { 6 } else { 1 };
    let x_mask = if dx < 0 {
        CollisionFlag::BlockWest as u32
    } else {
        CollisionFlag::BlockEast as u32
    } | extra;
    let z_mask = if dz < 0 {
        CollisionFlag::BlockSouth as u32
    } else {
        CollisionFlag::BlockNorth as u32
    } | extra;
    unsafe {
        let flags = &*COLLISION_FLAGS;
        if dx == 0 {
            return if dz == 0 {
                -1
            } else if flags.get(x, z + dz as i32, y) & z_mask == 0 {
                z_dir
            } else {
                -2
            };
        }
        let x_clear = flags.get(x + dx as i32, z, y) & x_mask == 0;
        if dz == 0 {
            return if x_clear { x_dir } else { -2 };
        }
        if x_clear {
            let (diagonal, dir) = match (dx, dz) {
                (-1, -1) => (CollisionFlag::BlockSouthWest as u32, 5),
                (-1, 1) => (CollisionFlag::BlockNorthWest as u32, 0),
                (1, -1) => (CollisionFlag::BlockSouthEast as u32, 7),
                (1, 1) => (CollisionFlag::BlockNorthEast as u32, 2),
                _ => unreachable!(),
            };
            if flags.get(x + dx as i32, z + dz as i32, y) & (diagonal | extra) == 0
                && flags.get(x, z + dz as i32, y) & z_mask == 0
            {
                return dir;
            }
            return x_dir;
        }
        if flags.get(x, z + dz as i32, y) & z_mask == 0 {
            z_dir
        } else {
            -2
        }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn has_line_of_sight(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_width: u8,
    src_length: u8,
    dest_width: u8,
    dest_length: u8,
    extra_flag: u32,
) -> bool {
    unsafe {
        rsmod::line_validator::has_line_of_sight(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_width,
            src_length,
            dest_width,
            dest_length,
            extra_flag,
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub fn has_line_of_walk(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_width: u8,
    src_length: u8,
    dest_width: u8,
    dest_length: u8,
    extra_flag: u32,
) -> bool {
    unsafe {
        rsmod::line_validator::has_line_of_walk(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_width,
            src_length,
            dest_width,
            dest_length,
            extra_flag,
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub fn line_of_sight(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_width: u8,
    src_length: u8,
    dest_width: u8,
    dest_length: u8,
    extra_flag: u32,
) -> &'static [u32] {
    unsafe {
        rsmod::line_pathfinder::line_of_sight(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_width,
            src_length,
            dest_width,
            dest_length,
            extra_flag,
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub fn line_of_walk(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    src_width: u8,
    src_length: u8,
    dest_width: u8,
    dest_length: u8,
    extra_flag: u32,
) -> &'static [u32] {
    unsafe {
        rsmod::line_pathfinder::line_of_walk(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            src_width,
            src_length,
            dest_width,
            dest_length,
            extra_flag,
        )
    }
}

#[allow(clippy::too_many_arguments)]
pub fn reached(
    y: u8,
    src_x: u16,
    src_z: u16,
    dest_x: u16,
    dest_z: u16,
    dest_width: u8,
    dest_length: u8,
    src_size: u8,
    angle: u8,
    shape: i8,
    block_access_flags: u8,
) -> bool {
    unsafe {
        ReachStrategy::reached(
            &COLLISION_FLAGS,
            y as i32,
            src_x as i32,
            src_z as i32,
            dest_x as i32,
            dest_z as i32,
            dest_width,
            dest_length,
            src_size,
            angle,
            shape,
            block_access_flags,
        )
    }
}

pub fn __set(x: u16, z: u16, y: u8, mask: u32) {
    unsafe {
        COLLISION_FLAGS.set(x as i32, z as i32, y as i32, mask);
    }
}

#[test]
fn normal_one_matches_all_fallbacks() {
    let mut seed = 0x987654321abcdefu64;
    let mut rnd = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 32) as u32
    };
    for i in 0..100000 {
        let x = [0u16, 7, 8, 63, 64, 3200, 3207, 16383, 65535][rnd() as usize % 9];
        let z = [0u16, 7, 8, 63, 64, 3200, 3207, 16383, 65535][rnd() as usize % 9];
        let y = (rnd() % 4) as u8;
        for ox in -1i32..=1 {
            for oz in -1i32..=1 {
                __set(
                    (x as i32 + ox) as u16,
                    (z as i32 + oz) as u16,
                    y,
                    rnd() & rnd() & rnd(),
                );
            }
        }
        for dx in -1i8..=1 {
            for dz in -1i8..=1 {
                for extra in [0, CollisionFlag::Npc as u32, CollisionFlag::Player as u32] {
                    let reference = if dx == 0 && dz == 0 {
                        -1
                    } else if can_travel(y, x, z, dx, dz, 1, extra, CollisionType::Normal) {
                        match (dx, dz) {
                            (-1, 1) => 0,
                            (0, 1) => 1,
                            (1, 1) => 2,
                            (-1, 0) => 3,
                            (1, 0) => 4,
                            (-1, -1) => 5,
                            (0, -1) => 6,
                            (1, -1) => 7,
                            _ => unreachable!(),
                        }
                    } else if dx != 0 && can_travel(y, x, z, dx, 0, 1, extra, CollisionType::Normal)
                    {
                        if dx < 0 { 3 } else { 4 }
                    } else if dz != 0 && can_travel(y, x, z, 0, dz, 1, extra, CollisionType::Normal)
                    {
                        if dz < 0 { 6 } else { 1 }
                    } else {
                        -2
                    };
                    assert_eq!(
                        can_travel_fast(y, x, z, dx, dz, extra),
                        reference,
                        "{i} {x} {z} {dx} {dz}"
                    );
                }
            }
        }
    }
}
