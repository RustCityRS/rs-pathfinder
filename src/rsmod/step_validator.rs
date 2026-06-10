use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::CollisionStrategy;
use crate::rsmod::flag::collision_flag::CollisionFlag;

#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) unsafe fn can_travel<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    offset_x: i8,
    offset_z: i8,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match (offset_x, offset_z) {
        (0, -1) => !is_blocked_south(flags, y, x, z, size, extra_flag, collision),
        (0, 1) => !is_blocked_north(flags, y, x, z, size, extra_flag, collision),
        (-1, 0) => !is_blocked_west(flags, y, x, z, size, extra_flag, collision),
        (1, 0) => !is_blocked_east(flags, y, x, z, size, extra_flag, collision),
        (-1, -1) => !is_blocked_southwest(flags, y, x, z, size, extra_flag, collision),
        (-1, 1) => !is_blocked_northwest(flags, y, x, z, size, extra_flag, collision),
        (1, -1) => !is_blocked_southeast(flags, y, x, z, size, extra_flag, collision),
        (1, 1) => !is_blocked_northeast(flags, y, x, z, size, extra_flag, collision),
        _ => false,
    }
}

#[inline(always)]
unsafe fn is_blocked_south<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => !collision.can_move(
            flags.get(x, z - 1, y),
            CollisionFlag::BlockSouth as u32 | extra_flag,
        ),
        2 => {
            !collision.can_move(
                flags.get(x, z - 1, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 1, z - 1, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x, z - 1, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + size as i32 - 1, z - 1, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in x + 1..x + size as i32 - 1 {
                if !collision.can_move(
                    flags.get(mid, z - 1, y),
                    CollisionFlag::BlockNorthEastAndWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_north<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => !collision.can_move(
            flags.get(x, z + 1, y),
            CollisionFlag::BlockNorth as u32 | extra_flag,
        ),
        2 => {
            !collision.can_move(
                flags.get(x, z + 2, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 1, z + 2, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x, z + size as i32, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + size as i32 - 1, z + size as i32, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in x + 1..x + size as i32 - 1 {
                if !collision.can_move(
                    flags.get(mid, z + size as i32, y),
                    CollisionFlag::BlockSouthEastAndWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_west<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => !collision.can_move(
            flags.get(x - 1, z, y),
            CollisionFlag::BlockWest as u32 | extra_flag,
        ),
        2 => {
            !collision.can_move(
                flags.get(x - 1, z, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z + 1, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x - 1, z, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z + size as i32 - 1, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in z + 1..z + size as i32 - 1 {
                if !collision.can_move(
                    flags.get(x - 1, mid, y),
                    CollisionFlag::BlockNorthAndSouthEast as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_east<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => !collision.can_move(
            flags.get(x + 1, z, y),
            CollisionFlag::BlockEast as u32 | extra_flag,
        ),
        2 => {
            !collision.can_move(
                flags.get(x + 2, z, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 2, z + 1, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x + size as i32, z, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + size as i32, z + size as i32 - 1, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in z + 1..z + size as i32 - 1 {
                if !collision.can_move(
                    flags.get(x + size as i32, mid, y),
                    CollisionFlag::BlockNorthAndSouthWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_southwest<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => {
            !collision.can_move(
                flags.get(x - 1, z - 1, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z, y),
                CollisionFlag::BlockWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z - 1, y),
                CollisionFlag::BlockSouth as u32 | extra_flag,
            )
        }
        2 => {
            !collision.can_move(
                flags.get(x - 1, z, y),
                CollisionFlag::BlockNorthAndSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z - 1, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z - 1, y),
                CollisionFlag::BlockNorthEastAndWest as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x - 1, z - 1, y),
                CollisionFlag::BlockSouthWest as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in 1..size {
                if !collision.can_move(
                    flags.get(x - 1, z + mid as i32 - 1, y),
                    CollisionFlag::BlockNorthAndSouthEast as u32 | extra_flag,
                ) || !collision.can_move(
                    flags.get(x + mid as i32 - 1, z - 1, y),
                    CollisionFlag::BlockNorthEastAndWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_northwest<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => {
            !collision.can_move(
                flags.get(x - 1, z + 1, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z, y),
                CollisionFlag::BlockWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z + 1, y),
                CollisionFlag::BlockNorth as u32 | extra_flag,
            )
        }
        2 => {
            !collision.can_move(
                flags.get(x - 1, z + 1, y),
                CollisionFlag::BlockNorthAndSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x - 1, z + 2, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z + 2, y),
                CollisionFlag::BlockSouthEastAndWest as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x - 1, z + size as i32, y),
                CollisionFlag::BlockNorthWest as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in 1..size {
                if !collision.can_move(
                    flags.get(x - 1, z + mid as i32, y),
                    CollisionFlag::BlockNorthAndSouthEast as u32 | extra_flag,
                ) || !collision.can_move(
                    flags.get(x + mid as i32 - 1, z + size as i32, y),
                    CollisionFlag::BlockSouthEastAndWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_southeast<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => {
            !collision.can_move(
                flags.get(x + 1, z - 1, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 1, z, y),
                CollisionFlag::BlockEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z - 1, y),
                CollisionFlag::BlockSouth as u32 | extra_flag,
            )
        }
        2 => {
            !collision.can_move(
                flags.get(x + 1, z - 1, y),
                CollisionFlag::BlockNorthEastAndWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 2, z - 1, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 2, z, y),
                CollisionFlag::BlockNorthAndSouthWest as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x + size as i32, z - 1, y),
                CollisionFlag::BlockSouthEast as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in 1..size {
                if !collision.can_move(
                    flags.get(x + size as i32, z + mid as i32 - 1, y),
                    CollisionFlag::BlockNorthAndSouthWest as u32 | extra_flag,
                ) || !collision.can_move(
                    flags.get(x + mid as i32, z - 1, y),
                    CollisionFlag::BlockNorthEastAndWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[inline(always)]
unsafe fn is_blocked_northeast<C: CollisionStrategy>(
    flags: &CollisionFlagMap,
    y: i32,
    x: i32,
    z: i32,
    size: u8,
    extra_flag: u32,
    collision: &C,
) -> bool {
    match size {
        1 => {
            !collision.can_move(
                flags.get(x + 1, z + 1, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 1, z, y),
                CollisionFlag::BlockEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x, z + 1, y),
                CollisionFlag::BlockNorth as u32 | extra_flag,
            )
        }
        2 => {
            !collision.can_move(
                flags.get(x + 1, z + 2, y),
                CollisionFlag::BlockSouthEastAndWest as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 2, z + 2, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            ) || !collision.can_move(
                flags.get(x + 2, z + 1, y),
                CollisionFlag::BlockNorthAndSouthWest as u32 | extra_flag,
            )
        }
        _ => {
            if !collision.can_move(
                flags.get(x + size as i32, z + size as i32, y),
                CollisionFlag::BlockNorthEast as u32 | extra_flag,
            ) {
                return true;
            }
            for mid in 1..size {
                if !collision.can_move(
                    flags.get(x + mid as i32, z + size as i32, y),
                    CollisionFlag::BlockSouthEastAndWest as u32 | extra_flag,
                ) || !collision.can_move(
                    flags.get(x + size as i32, z + mid as i32, y),
                    CollisionFlag::BlockNorthAndSouthWest as u32 | extra_flag,
                ) {
                    return true;
                }
            }
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::collision::collision_strategy::{
        Blocked, Indoors, LineOfSight, Normal, Outdoors,
    };
    use crate::rsmod::flag::collision_flag::CollisionFlag;
    use crate::rsmod::step_validator::can_travel;

    const ARGS: [[i32; 3]; 24] = [
        [1, 0, -1],
        [1, 0, 1],
        [1, -1, 0],
        [1, 1, 0],
        [1, -1, -1],
        [1, -1, 1],
        [1, 1, -1],
        [1, 1, 1],
        [2, 0, -1],
        [2, 0, 1],
        [2, -1, 0],
        [2, 1, 0],
        [2, -1, -1],
        [2, -1, 1],
        [2, 1, -1],
        [2, 1, 1],
        [3, 0, -1],
        [3, 0, 1],
        [3, -1, 0],
        [3, 1, 0],
        [3, -1, -1],
        [3, -1, 1],
        [3, 1, -1],
        [3, 1, 1],
    ];

    unsafe fn build_collision_map(x1: i32, z1: i32, x2: i32, z2: i32) -> CollisionFlagMap {
        let mut collision = CollisionFlagMap::new();
        for y in 0..4 {
            for z in z1.min(z2)..=z1.max(z2) {
                for x in x1.min(x2)..=x1.max(x2) {
                    collision.allocate_if_absent(x, z, y);
                }
            }
        }
        return collision;
    }

    unsafe fn build_collision_map_with_flag(
        x1: i32,
        z1: i32,
        x2: i32,
        z2: i32,
        mask: CollisionFlag,
    ) -> CollisionFlagMap {
        let mut collision = CollisionFlagMap::new();
        for y in 0..4 {
            for z in z1.min(z2)..=z1.max(z2) {
                for x in x1.min(x2)..=x1.max(x2) {
                    collision.set(x, z, y, mask as u32);
                }
            }
        }
        return collision;
    }

    #[test]
    fn test_step_clear_path() {
        for [size, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;

            unsafe {
                let collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    let step = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        size as u8,
                        0,
                        &Normal,
                    );
                    assert!(step);
                }
            }
        }
    }

    #[test]
    fn test_step_loc_blocking() {
        for [size, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    let area = size * size;
                    for index in 0..area {
                        let dx: i32 = dest_x + (index % size);
                        let dz: i32 = dest_z + (index / size);
                        collision.set(dx, dz, y, CollisionFlag::Loc as u32);
                    }
                }
                for y in 0..4 {
                    let step = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        size as u8,
                        0,
                        &Normal,
                    );
                    assert!(!step);
                }
            }
        }
    }

    #[test]
    fn test_step_extra_flag_blocking() {
        let flags = [
            CollisionFlag::Player as u32,
            CollisionFlag::Npc as u32,
            CollisionFlag::Player as u32 | CollisionFlag::Npc as u32,
        ];

        for [size, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);
                for y in 0..4 {
                    let area = size * size;
                    for flag in flags {
                        for index in 0..area {
                            let dx: i32 = dest_x + (index % size);
                            let dz: i32 = dest_z + (index / size);
                            collision.set(dx, dz, y, flag);
                        }
                    }
                }
                for y in 0..4 {
                    for flag in flags {
                        let step = can_travel(
                            &collision,
                            y,
                            src_x,
                            src_z,
                            dir_x as i8,
                            dir_z as i8,
                            size as u8,
                            flag,
                            &Normal,
                        );
                        assert!(!step);
                    }
                }
            }
        }
    }

    #[test]
    fn test_step_blocked_flag_strategy() {
        for [_, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;

            unsafe {
                let collision = build_collision_map_with_flag(
                    src_x,
                    src_z,
                    dest_x,
                    dest_z,
                    CollisionFlag::Floor,
                );
                for y in 0..4 {
                    let step = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &Blocked,
                    );
                    assert!(step);
                }
            }
        }
    }

    #[test]
    fn test_step_indoors_flag_strategy() {
        for [_, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;
            let outdoors_x = dest_x + dir_x;
            let outdoors_z = dest_z + dir_z;

            let mut collision = CollisionFlagMap::new();

            unsafe {
                for y in 0..4 {
                    for x in src_x.min(dest_x.min(outdoors_x))..=src_x.max(dest_x.max(outdoors_x)) {
                        for z in
                            src_z.min(dest_z.min(outdoors_z))..=src_z.max(dest_z.max(outdoors_z))
                        {
                            collision.set(x, z, y, CollisionFlag::Roof as u32);
                        }
                    }
                }

                for y in 0..4 {
                    collision.set(outdoors_x, outdoors_z, y, CollisionFlag::Open as u32);
                }

                for y in 0..4 {
                    let step1 = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &Indoors,
                    );
                    assert!(step1);

                    let step2 = can_travel(
                        &collision,
                        y,
                        dest_x,
                        dest_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &Indoors,
                    );
                    assert!(!step2);
                }
            }
        }
    }

    #[test]
    fn test_step_outdoors_flag_strategy() {
        for [_, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;
            let indoors_x = dest_x + dir_x;
            let indoors_z = dest_z + dir_z;

            let mut collision = CollisionFlagMap::new();

            unsafe {
                for y in 0..4 {
                    for x in src_x.min(dest_x.min(indoors_x))..=src_x.max(dest_x.max(indoors_x)) {
                        for z in src_z.min(dest_z.min(indoors_z))..=src_z.max(dest_z.max(indoors_z))
                        {
                            collision.allocate_if_absent(x, z, y);
                        }
                    }
                }

                for y in 0..4 {
                    collision.set(indoors_x, indoors_z, y, CollisionFlag::Roof as u32);
                }

                for y in 0..4 {
                    let step1 = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &Outdoors,
                    );
                    assert!(step1);

                    let step2 = can_travel(
                        &collision,
                        y,
                        dest_x,
                        dest_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &Outdoors,
                    );
                    assert!(!step2);
                }
            }
        }
    }

    #[test]
    fn test_step_line_of_sight_strategy_loc() {
        for [_, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;
            let blocked_x = dest_x + dir_x;
            let blocked_z = dest_z + dir_z;

            let mut collision = CollisionFlagMap::new();

            unsafe {
                for y in 0..4 {
                    for x in src_x.min(dest_x.min(blocked_x))..=src_x.max(dest_x.max(blocked_x)) {
                        for z in src_z.min(dest_z.min(blocked_z))..=src_z.max(dest_z.max(blocked_z))
                        {
                            collision.allocate_if_absent(x, z, y);
                        }
                    }
                }

                for y in 0..4 {
                    collision.set(
                        blocked_x,
                        blocked_z,
                        y,
                        CollisionFlag::LocProjBlocker as u32,
                    );
                }

                for y in 0..4 {
                    let step1 = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &LineOfSight,
                    );
                    assert!(step1);

                    let step2 = can_travel(
                        &collision,
                        y,
                        dest_x,
                        dest_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &LineOfSight,
                    );
                    assert!(!step2);
                }
            }
        }
    }

    #[test]
    fn test_step_line_of_sight_strategy_player() {
        for [_, dir_x, dir_z] in ARGS {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = src_x + dir_x;
            let dest_z = src_z + dir_z;
            let blocked_x = dest_x + dir_x;
            let blocked_z = dest_z + dir_z;

            let mut collision = CollisionFlagMap::new();

            unsafe {
                for y in 0..4 {
                    for x in src_x.min(dest_x.min(blocked_x))..=src_x.max(dest_x.max(blocked_x)) {
                        for z in src_z.min(dest_z.min(blocked_z))..=src_z.max(dest_z.max(blocked_z))
                        {
                            collision.allocate_if_absent(x, z, y);
                        }
                    }
                }

                for y in 0..4 {
                    collision.set(blocked_x, blocked_z, y, CollisionFlag::Player as u32);
                }

                for y in 0..4 {
                    let step1 = can_travel(
                        &collision,
                        y,
                        src_x,
                        src_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &LineOfSight,
                    );
                    assert!(step1);

                    let step2 = can_travel(
                        &collision,
                        y,
                        dest_x,
                        dest_z,
                        dir_x as i8,
                        dir_z as i8,
                        1,
                        0,
                        &LineOfSight,
                    );
                    assert!(step2);
                }
            }
        }
    }
}
