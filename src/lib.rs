#![allow(static_mut_refs)]
#![allow(unsafe_op_in_unsafe_fn)]

use once_cell::sync::Lazy;

use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::*;
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
    let mask: u32 = CollisionFlag::Npc as u32;
    let x = x as i32;
    let z = z as i32;
    let y = y as i32;
    let size = size as i32;
    let area = size * size;
    unsafe {
        if add {
            for index in 0..area {
                COLLISION_FLAGS.add(x + (index % size), z + (index / size), y, mask);
            }
        } else {
            for index in 0..area {
                COLLISION_FLAGS.remove(x + (index % size), z + (index / size), y, mask);
            }
        }
    }
}

pub fn change_player(x: u16, z: u16, y: u8, size: u8, add: bool) {
    let mask: u32 = CollisionFlag::Player as u32;
    let x = x as i32;
    let z = z as i32;
    let y = y as i32;
    let size = size as i32;
    let area = size * size;
    unsafe {
        if add {
            for index in 0..area {
                COLLISION_FLAGS.add(x + (index % size), z + (index / size), y, mask);
            }
        } else {
            for index in 0..area {
                COLLISION_FLAGS.remove(x + (index % size), z + (index / size), y, mask);
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
