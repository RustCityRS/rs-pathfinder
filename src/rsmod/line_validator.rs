use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::flag::collision_flag::CollisionFlag;
use crate::rsmod::line::Line;

#[allow(clippy::too_many_arguments)]
pub(crate) unsafe fn has_line_of_sight(
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
    extra_flag: u32,
) -> bool {
    ray_cast_line(
        flags,
        y,
        src_x,
        src_z,
        dest_x,
        dest_z,
        src_width,
        src_height,
        dest_width,
        dest_height,
        Line::SIGHT_BLOCKED_WEST | extra_flag,
        Line::SIGHT_BLOCKED_EAST | extra_flag,
        Line::SIGHT_BLOCKED_SOUTH | extra_flag,
        Line::SIGHT_BLOCKED_NORTH | extra_flag,
        CollisionFlag::Loc as u32 | extra_flag,
        CollisionFlag::LocProjBlocker as u32 | extra_flag,
        true,
    )
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) unsafe fn has_line_of_walk(
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
    extra_flag: u32,
) -> bool {
    ray_cast_line(
        flags,
        y,
        src_x,
        src_z,
        dest_x,
        dest_z,
        src_width,
        src_height,
        dest_width,
        dest_height,
        Line::WALK_BLOCKED_WEST | extra_flag,
        Line::WALK_BLOCKED_EAST | extra_flag,
        Line::WALK_BLOCKED_SOUTH | extra_flag,
        Line::WALK_BLOCKED_NORTH | extra_flag,
        CollisionFlag::Loc as u32 | extra_flag,
        CollisionFlag::LocProjBlocker as u32 | extra_flag,
        false,
    )
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
unsafe fn ray_cast_line(
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
    flag_west: u32,
    flag_east: u32,
    flag_south: u32,
    flag_north: u32,
    flag_loc: u32,
    flag_proj: u32,
    los: bool,
) -> bool {
    let start_x: i32 = Line::coordinate(src_x, dest_x, src_width);
    let start_z: i32 = Line::coordinate(src_z, dest_z, src_height);

    let end_x: i32 = Line::coordinate(dest_x, src_x, dest_width);
    let end_z: i32 = Line::coordinate(dest_z, src_z, dest_height);

    if start_x == end_x && start_z == end_z {
        return true;
    }

    if los && flags.is_flagged(start_x, start_z, y, flag_loc) {
        return false;
    }

    let delta_x: i32 = end_x - start_x;
    let delta_z: i32 = end_z - start_z;
    let absolute_delta_x: i32 = delta_x.abs();
    let absolute_delta_z: i32 = delta_z.abs();

    let travel_east: bool = delta_x >= 0;
    let travel_north: bool = delta_z >= 0;

    let mut x_flags: u32 = if travel_east { flag_west } else { flag_east };
    let mut z_flags: u32 = if travel_north { flag_south } else { flag_north };

    if absolute_delta_x > absolute_delta_z {
        let offset_x: i32 = if travel_east { 1 } else { -1 };
        let offset_z: i32 = if travel_north { 0 } else { -1 };

        let mut scaled_z: i32 = Line::scale_up(start_z) + Line::HALF_TILE + offset_z;
        let tangent: i32 = Line::scale_up(delta_z) / absolute_delta_x;

        let mut curr_x: i32 = start_x;
        while curr_x != end_x {
            curr_x += offset_x;
            let curr_z: i32 = Line::scale_down(scaled_z);
            if los && curr_x == end_x && curr_z == end_z {
                x_flags &= !flag_proj;
            }
            if flags.is_flagged(curr_x, curr_z, y, x_flags) {
                return false;
            }

            scaled_z += tangent;

            let next_z: i32 = Line::scale_down(scaled_z);
            if los && curr_x == end_x && next_z == end_z {
                z_flags &= !flag_proj;
            }
            if next_z != curr_z && flags.is_flagged(curr_x, next_z, y, z_flags) {
                return false;
            }
        }
    } else {
        let offset_x: i32 = if travel_east { 0 } else { -1 };
        let offset_z: i32 = if travel_north { 1 } else { -1 };

        let mut scaled_x: i32 = Line::scale_up(start_x) + Line::HALF_TILE + offset_x;
        let tangent: i32 = Line::scale_up(delta_x) / absolute_delta_z;

        let mut curr_z: i32 = start_z;
        while curr_z != end_z {
            curr_z += offset_z;
            let curr_x: i32 = Line::scale_down(scaled_x);
            if los && curr_x == end_x && curr_z == end_z {
                z_flags &= !flag_proj;
            }
            if flags.is_flagged(curr_x, curr_z, y, z_flags) {
                return false;
            }

            scaled_x += tangent;

            let next_x: i32 = Line::scale_down(scaled_x);
            if los && next_x == end_x && curr_z == end_z {
                x_flags &= !flag_proj;
            }
            if next_x != curr_x && flags.is_flagged(next_x, curr_z, y, x_flags) {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::flag::collision_flag::CollisionFlag;
    use crate::rsmod::line_validator::{has_line_of_sight, has_line_of_walk};

    const ARGS: [[i32; 2]; 4] = [[0, -1], [0, 1], [-1, 0], [1, 0]];

    unsafe fn build_collision_map(x1: i32, z1: i32, x2: i32, z2: i32) -> CollisionFlagMap {
        let mut collision = CollisionFlagMap::new();
        for y in 0..4 {
            for z in z1.min(z2)..=z1.max(z2) {
                for x in x1.min(x2)..=x1.max(x2) {
                    collision.allocate_if_absent(x, z, y);
                }
            }
        }
        collision
    }

    #[test]
    fn test_has_low_partial_line_of_walk() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(3200, 3205, 0, CollisionFlag::Loc as u32);

            let line = has_line_of_walk(&collision, 0, src_x, src_z, 3200, 3207, 1, 1, 0, 0, 0);
            assert!(!line);
        }
    }

    #[test]
    fn test_has_low_clear_line_of_walk() {
        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    collision.allocate_if_absent(src_x + dir_x, src_z + dir_z, y);

                    let line = has_line_of_walk(
                        &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, 0,
                    );
                    assert!(line);
                }
            }
        }
    }

    #[test]
    fn test_has_low_loc_blocking() {
        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    collision.set(src_x + dir_x, src_z + dir_z, y, CollisionFlag::Loc as u32);

                    let line = has_line_of_walk(
                        &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, 0,
                    );
                    assert!(!line);
                }
            }
        }
    }

    #[test]
    fn test_has_low_extra_flag_blocking() {
        let flags = [
            CollisionFlag::Player as u32,
            CollisionFlag::Npc as u32,
            CollisionFlag::Player as u32 | CollisionFlag::Npc as u32,
        ];

        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    for flag in flags {
                        collision.set(src_x + dir_x, src_z + dir_z, y, flag);

                        let line = has_line_of_walk(
                            &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, flag,
                        );
                        assert!(!line);
                    }
                }
            }
        }
    }

    #[test]
    fn test_has_los_with_extra_flag_on_target_coords() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.add(src_x, src_z, 0, CollisionFlag::Player as u32);

            let line = has_line_of_sight(
                &collision,
                0,
                3200,
                3202,
                3200,
                3200,
                1,
                1,
                1,
                1,
                CollisionFlag::Player as u32,
            );
            assert!(line);
        }
    }

    #[test]
    fn test_has_los_with_extra_flag_past_target_coords() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.add(src_x, src_z, 0, CollisionFlag::Player as u32);

            let line = has_line_of_sight(
                &collision,
                0,
                3200,
                3202,
                3200,
                3199,
                1,
                1,
                1,
                1,
                CollisionFlag::Player as u32,
            );
            assert!(!line);
        }
    }

    #[test]
    fn test_has_los_on_top_of_loc_fails_line_of_sight() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.add(src_x, src_z, 0, CollisionFlag::Loc as u32);

            let line = has_line_of_sight(&collision, 0, src_x, src_z, 3200, 3201, 1, 1, 0, 0, 0);
            assert!(!line);
        }
    }

    #[test]
    fn test_has_los_on_top_of_extra_flag_fails_line_of_sight() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.add(src_x, src_z, 0, CollisionFlag::Player as u32);

            let line = has_line_of_sight(
                &collision,
                0,
                src_x,
                src_z,
                3200,
                3201,
                1,
                1,
                0,
                0,
                CollisionFlag::Player as u32,
            );
            assert!(!line);
        }
    }

    // #[test]
    // fn test_has_los_same_tile_has_line_of_sight() {
    //     let src_x = 3200;
    //     let src_z = 3200;
    //
    //     let mut collision = CollisionFlagMap::new();
    //
    //     unsafe {
    //         collision.allocate_if_absent(src_x, src_z, 0);
    //
    //         let line = has_line_of_sight(&collision, 0, src_x, src_z, src_x, src_z, 1, 1, 0, 0, 0);
    //         assert!(line.len() > 0);
    //     }
    // }

    #[test]
    fn test_has_los_partial_line_of_sight() {
        let src_x = 3200;
        let src_z = 3200;

        let mut collision = CollisionFlagMap::new();

        unsafe {
            collision.set(3200, 3205, 0, CollisionFlag::LocProjBlocker as u32);

            let line = has_line_of_sight(&collision, 0, src_x, src_z, 3200, 3207, 1, 1, 0, 0, 0);
            assert!(!line);
        }
    }

    #[test]
    fn test_has_los_valid_line_of_sight() {
        let flags = [
            CollisionFlag::Open as u32,
            CollisionFlag::Floor as u32,
            CollisionFlag::FloorDecoration as u32,
            CollisionFlag::Floor as u32 | CollisionFlag::FloorDecoration as u32,
        ];

        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    for flag in flags {
                        collision.set(src_x + dir_x, src_z + dir_z, y, flag);
                    }
                    let line = has_line_of_sight(
                        &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, 0,
                    );
                    assert!(line);
                }
            }
        }
    }

    #[test]
    fn test_has_los_loc_blocking() {
        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    collision.set(
                        src_x + dir_x,
                        src_z + dir_z,
                        y,
                        CollisionFlag::LocProjBlocker as u32,
                    );
                    let line = has_line_of_sight(
                        &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, 0,
                    );
                    assert!(!line);
                }
            }
        }
    }

    #[test]
    fn test_has_los_extra_flag_blocking() {
        let flags = [
            CollisionFlag::Player as u32,
            CollisionFlag::Npc as u32,
            CollisionFlag::Player as u32 | CollisionFlag::Npc as u32,
        ];

        for [dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x * 3;
            let dest_z = src_z + dir_z * 3;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    for flag in flags {
                        collision.set(src_x + dir_x, src_z + dir_z, y, flag);

                        let line = has_line_of_sight(
                            &collision, y, src_x, src_z, dest_x, dest_z, 1, 1, 0, 0, flag,
                        );
                        assert!(!line);
                    }
                }
            }
        }
    }
}
