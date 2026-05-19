use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::flag::block_flag::BlockAccessFlag;
use crate::rsmod::flag::collision_flag::CollisionFlag;

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub fn collides(
    src_x: i32,
    src_z: i32,
    dest_x: i32,
    dest_z: i32,
    src_width: u8,
    src_height: u8,
    dest_width: u8,
    dest_height: u8,
) -> bool {
    if src_x >= dest_x + dest_width as i32 || src_x + src_width as i32 <= dest_x {
        false
    } else {
        src_z < dest_z + dest_height as i32 && dest_z < src_height as i32 + src_z
    }
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) unsafe fn reach_rectangle_1(
    flags: &CollisionFlagMap,
    y: i32,
    src_x: i32,
    src_z: i32,
    dest_x: i32,
    dest_z: i32,
    dest_width: u8,
    dest_height: u8,
    block_access_flags: u8,
) -> bool {
    let east: i32 = dest_x + dest_width as i32 - 1;
    let north: i32 = dest_z + dest_height as i32 - 1;

    if src_x == dest_x - 1
        && src_z >= dest_z
        && src_z <= north
        && (flags.get(src_x, src_z, y) & CollisionFlag::WallEast as u32)
            == CollisionFlag::Open as u32
        && (block_access_flags & BlockAccessFlag::West) == 0
    {
        return true;
    }

    if src_x == east + 1
        && src_z >= dest_z
        && src_z <= north
        && (flags.get(src_x, src_z, y) & CollisionFlag::WallWest as u32)
            == CollisionFlag::Open as u32
        && (block_access_flags & BlockAccessFlag::East) == 0
    {
        return true;
    }

    if src_z + 1 == dest_z
        && src_x >= dest_x
        && src_x <= east
        && (flags.get(src_x, src_z, y) & CollisionFlag::WallNorth as u32)
            == CollisionFlag::Open as u32
        && (block_access_flags & BlockAccessFlag::South) == 0
    {
        return true;
    }

    src_z == north + 1
        && src_x >= dest_x
        && src_x <= east
        && (flags.get(src_x, src_z, y) & CollisionFlag::WallSouth as u32)
            == CollisionFlag::Open as u32
        && (block_access_flags & BlockAccessFlag::North) == 0
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) unsafe fn reach_rectangle_n(
    flags: &CollisionFlagMap,
    y: i32,
    src_x: i32,
    src_z: i32,
    dest_x: i32,
    dest_z: i32,
    src_width: u8,
    src_height: u8,
    dest_width: u8,
    dest_height: u8,
    block_access_flags: u8,
) -> bool {
    let src_east: i32 = src_x + src_width as i32;
    let src_north: i32 = src_height as i32 + src_z;
    let dest_east: i32 = dest_width as i32 + dest_x;
    let dest_north: i32 = dest_height as i32 + dest_z;

    if dest_east == src_x && (block_access_flags & BlockAccessFlag::East) == 0 {
        let from_z: i32 = src_z.max(dest_z);
        let to_z: i32 = src_north.min(dest_north);
        for side_z in from_z..to_z {
            if (flags.get(dest_east - 1, side_z, y) & CollisionFlag::WallEast as u32)
                == CollisionFlag::Open as u32
            {
                return true;
            }
        }
    } else if src_east == dest_x && (block_access_flags & BlockAccessFlag::West) == 0 {
        let from_z: i32 = src_z.max(dest_z);
        let to_z: i32 = src_north.min(dest_north);
        for side_z in from_z..to_z {
            if (flags.get(dest_x, side_z, y) & CollisionFlag::WallWest as u32)
                == CollisionFlag::Open as u32
            {
                return true;
            }
        }
    } else if src_z == dest_north && (block_access_flags & BlockAccessFlag::North) == 0 {
        let from_x: i32 = src_x.max(dest_x);
        let to_x: i32 = src_east.min(dest_east);
        for side_x in from_x..to_x {
            if (flags.get(side_x, dest_north - 1, y) & CollisionFlag::WallNorth as u32)
                == CollisionFlag::Open as u32
            {
                return true;
            }
        }
    } else if dest_z == src_north && (block_access_flags & BlockAccessFlag::South) == 0 {
        let from_x: i32 = src_x.max(dest_x);
        let to_x: i32 = src_east.min(dest_east);
        for side_x in from_x..to_x {
            if (flags.get(side_x, dest_z, y) & CollisionFlag::WallSouth as u32)
                == CollisionFlag::Open as u32
            {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::flag::block_flag::BlockAccessFlag;
    use crate::rsmod::flag::collision_flag::CollisionFlag;
    use crate::rsmod::reach::rectangle_boundary::{collides, reach_rectangle_1, reach_rectangle_n};

    #[test]
    fn rec_bound_n_no_flags_should_return_true_when_src_is_west_of_dest() {
        let src_x = 0;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_n_no_flags_should_return_true_when_src_is_east_of_dest() {
        let src_x = 4;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_n_no_flags_should_return_true_when_src_is_south_of_dest() {
        let src_x = 2;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_n_no_flags_should_return_true_when_src_is_north_of_dest() {
        let src_x = 2;
        let src_z = 0;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_n_block_flags_should_return_false_when_wall_is_east_of_src() {
        let src_x = 0;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallEast as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                BlockAccessFlag::West as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_n_block_flags_should_return_false_when_wall_is_west_of_src() {
        let src_x = 4;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallWest as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                BlockAccessFlag::East as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_n_block_flags_should_return_false_when_wall_is_south_of_src() {
        let src_x = 2;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouth as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                BlockAccessFlag::North as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_n_block_flags_should_return_false_when_wall_is_north_of_src() {
        let src_x = 2;
        let src_z = 0;
        let dest_x = 2;
        let dest_z = 2;
        let src_width = 2;
        let src_height = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorth as u32);

            let collides = collides(
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
            );
            assert!(!collides);

            let reached = reach_rectangle_n(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_width,
                src_height,
                dest_width,
                dest_height,
                BlockAccessFlag::South as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_no_flags_should_return_true_when_src_is_west_of_dest() {
        let src_x = 1;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);
            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_1_no_flags_should_return_true_when_src_is_east_of_dest() {
        let src_x = 4;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);
            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_1_no_flags_should_return_true_when_src_is_south_of_dest() {
        let src_x = 2;
        let src_z = 1;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);
            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_1_no_flags_should_return_true_when_src_is_north_of_dest() {
        let src_x = 2;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);
            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_wall_is_east_of_src() {
        let src_x = 1;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallEast as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                BlockAccessFlag::West as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_wall_is_west_of_src() {
        let src_x = 4;
        let src_z = 2;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallWest as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                BlockAccessFlag::East as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_wall_is_south_of_src() {
        let src_x = 2;
        let src_z = 1;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorth as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                BlockAccessFlag::South as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_wall_is_north_of_src() {
        let src_x = 2;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouth as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                BlockAccessFlag::North as u8,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_src_not_adjacent_to_dest() {
        let src_x = 5;
        let src_z = 5;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_src_is_within_dest_rect() {
        let src_x = 3;
        let src_z = 3;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(!reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_true_when_src_is_at_boundary_and_no_wall_blocking() {
        let src_x = 3;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                0,
            );
            assert!(reached);
        }
    }

    #[test]
    fn rec_bound_1_block_flags_should_return_false_when_src_is_at_boundary_and_is_wall_blocking() {
        let src_x = 3;
        let src_z = 4;
        let dest_x = 2;
        let dest_z = 2;
        let dest_width = 2;
        let dest_height = 2;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouth as u32);

            let reached = reach_rectangle_1(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                dest_width,
                dest_height,
                BlockAccessFlag::North as u8,
            );
            assert!(!reached);
        }
    }
}
