use crate::rsmod::flag::collision_flag::CollisionFlag;

pub struct Line;

impl Line {
    pub const SIGHT_BLOCKED_NORTH: u32 =
        CollisionFlag::LocProjBlocker as u32 | CollisionFlag::WallNorthProjBlocker as u32;
    pub const SIGHT_BLOCKED_EAST: u32 =
        CollisionFlag::LocProjBlocker as u32 | CollisionFlag::WallEastProjBlocker as u32;
    pub const SIGHT_BLOCKED_SOUTH: u32 =
        CollisionFlag::LocProjBlocker as u32 | CollisionFlag::WallSouthProjBlocker as u32;
    pub const SIGHT_BLOCKED_WEST: u32 =
        CollisionFlag::LocProjBlocker as u32 | CollisionFlag::WallWestProjBlocker as u32;

    pub const WALK_BLOCKED_NORTH: u32 =
        CollisionFlag::WallNorth as u32 | CollisionFlag::WalkBlocked as u32;
    pub const WALK_BLOCKED_EAST: u32 =
        CollisionFlag::WallEast as u32 | CollisionFlag::WalkBlocked as u32;
    pub const WALK_BLOCKED_SOUTH: u32 =
        CollisionFlag::WallSouth as u32 | CollisionFlag::WalkBlocked as u32;
    pub const WALK_BLOCKED_WEST: u32 =
        CollisionFlag::WallWest as u32 | CollisionFlag::WalkBlocked as u32;

    pub const HALF_TILE: i32 = (1 << 16) / 2;

    #[inline(always)]
    pub const fn scale_up(tiles: i32) -> i32 {
        tiles << 16
    }

    #[inline(always)]
    pub const fn scale_down(tiles: i32) -> i32 {
        tiles >> 16
    }

    #[inline(always)]
    pub const fn coordinate(a: i32, b: i32, size: u8) -> i32 {
        let upper = a + size as i32 - 1;
        let mut result = b;
        if upper < result {
            result = upper;
        }
        if a > result {
            result = a;
        }
        result
    }
}
