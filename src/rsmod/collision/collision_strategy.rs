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

pub trait CollisionStrategy {
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool;
}

pub struct Normal;
pub struct Blocked;
pub struct Indoors;
pub struct Outdoors;
pub struct LineOfSight;

impl CollisionStrategy for Normal {
    #[inline(always)]
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool {
        (tile_flag & block_flag) == CollisionFlag::Open as u32
    }
}

impl CollisionStrategy for Blocked {
    #[inline(always)]
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool {
        let flag = block_flag & !(CollisionFlag::Floor as u32);
        (tile_flag & flag) == CollisionFlag::Open as u32
            && (tile_flag & CollisionFlag::Floor as u32) != CollisionFlag::Open as u32
    }
}

impl CollisionStrategy for Indoors {
    #[inline(always)]
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool {
        (tile_flag & block_flag) == CollisionFlag::Open as u32
            && (tile_flag & CollisionFlag::Roof as u32) != CollisionFlag::Open as u32
    }
}

impl CollisionStrategy for Outdoors {
    #[inline(always)]
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool {
        (tile_flag & (block_flag | CollisionFlag::Roof as u32)) == CollisionFlag::Open as u32
    }
}

impl CollisionStrategy for LineOfSight {
    #[inline(always)]
    fn can_move(&self, tile_flag: u32, block_flag: u32) -> bool {
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
            | CollisionFlag::LocRouteBlocker as u32;
        let movement_flags = (block_flag & MOVEMENT) << 9;
        let route_flags = (block_flag & ROUTE) >> 13;
        (tile_flag & (movement_flags | route_flags)) == CollisionFlag::Open as u32
    }
}

/// Expands `$body` once per strategy with `$strategy` bound to a
/// reference to the matching zero-sized type, so each arm monomorphizes
/// the call graph below it.
#[macro_export]
macro_rules! with_collision_strategy {
    ($collision:expr, $strategy:ident => $body:expr) => {
        match $collision {
            CollisionType::Normal => {
                let $strategy = &$crate::rsmod::collision::collision_strategy::Normal;
                $body
            }
            CollisionType::Blocked => {
                let $strategy = &$crate::rsmod::collision::collision_strategy::Blocked;
                $body
            }
            CollisionType::Indoors => {
                let $strategy = &$crate::rsmod::collision::collision_strategy::Indoors;
                $body
            }
            CollisionType::Outdoors => {
                let $strategy = &$crate::rsmod::collision::collision_strategy::Outdoors;
                $body
            }
            CollisionType::LineOfSight => {
                let $strategy = &$crate::rsmod::collision::collision_strategy::LineOfSight;
                $body
            }
        }
    };
}
