#![allow(non_camel_case_types)]

use crate::rsmod::flag::collision_flag::CollisionFlag;

#[repr(u8)]
pub enum CollisionType {
    Normal = 0,
    Blocked = 1,
    Indoors = 2,
    Outdoors = 3,
    LineOfSight = 4,
}

pub type CollisionStrategy = fn(u32, u32) -> bool;

pub const fn normal_strategy(tile_flag: u32, block_flag: u32) -> bool {
    (tile_flag & block_flag) == CollisionFlag::Open as u32
}

pub const fn blocked_strategy(tile_flag: u32, block_flag: u32) -> bool {
    let flag = block_flag & !(CollisionFlag::Floor as u32);
    (tile_flag & flag) == CollisionFlag::Open as u32
        && (tile_flag & CollisionFlag::Floor as u32) != CollisionFlag::Open as u32
}

pub const fn indoors_strategy(tile_flag: u32, block_flag: u32) -> bool {
    (tile_flag & block_flag) == CollisionFlag::Open as u32
        && (tile_flag & CollisionFlag::Roof as u32) != CollisionFlag::Open as u32
}

pub const fn outdoors_strategy(tile_flag: u32, block_flag: u32) -> bool {
    (tile_flag & (block_flag | CollisionFlag::Roof as u32)) == CollisionFlag::Open as u32
}

pub const fn line_of_sight_strategy(tile_flag: u32, block_flag: u32) -> bool {
    const MOVEMENT: u32 = CollisionFlag::WallNorthWest as u32
        | CollisionFlag::WallNorth as u32
        | CollisionFlag::WallNorthEast as u32
        | CollisionFlag::WallEast as u32
        | CollisionFlag::WallSouthEast as u32
        | CollisionFlag::WallSouth as u32
        | CollisionFlag::WallSouthWest as u32
        | CollisionFlag::WallWest as u32
        | CollisionFlag::Loc as u32;
    const ROUTE: u32 = CollisionFlag::WallNorthWestRouteBlocker as u32
        | CollisionFlag::WallNorthRouteBlocker as u32
        | CollisionFlag::WallNorthEastRouteBlocker as u32
        | CollisionFlag::WallEastRouteBlocker as u32
        | CollisionFlag::WallSouthEastRouteBlocker as u32
        | CollisionFlag::WallSouthRouteBlocker as u32
        | CollisionFlag::WallSouthWestRouteBlocker as u32
        | CollisionFlag::WallWestRouteBlocker as u32
        | CollisionFlag::locRouteBlocker as u32;
    let movement_flags = (block_flag & MOVEMENT) << 9;
    let route_flags = (block_flag & ROUTE) >> 13;
    (tile_flag & (movement_flags | route_flags)) == CollisionFlag::Open as u32
}

#[inline(always)]
pub const fn get_collision_strategy(collision: CollisionType) -> CollisionStrategy {
    match collision {
        CollisionType::Normal => normal_strategy,
        CollisionType::Blocked => blocked_strategy,
        CollisionType::Indoors => indoors_strategy,
        CollisionType::Outdoors => outdoors_strategy,
        CollisionType::LineOfSight => line_of_sight_strategy,
    }
}
