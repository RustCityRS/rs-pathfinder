use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::flag::collision_flag::CollisionFlag;
use crate::rsmod::loc_angle::LocAngle;
use crate::rsmod::loc_shape::LocShape;
use crate::rsmod::reach::rectangle_boundary::{collides, reach_rectangle_1, reach_rectangle_n};
use crate::rsmod::utils::rotation::{rotate, rotate_flags};

pub(crate) struct ReachStrategy;

impl ReachStrategy {
    const WALL_STRATEGY: i32 = 0;
    const WALL_DECOR_STRATEGY: i32 = 1;
    const RECTANGLE_STRATEGY: i32 = 2;
    const NO_STRATEGY: i32 = 3;
    const RECTANGLE_EXCLUSIVE_STRATEGY: i32 = 4;

    #[inline(always)]
    const fn exit_strategy(shape: i8) -> i32 {
        if shape == -2 {
            return ReachStrategy::RECTANGLE_EXCLUSIVE_STRATEGY;
        } else if shape == -1 {
            return ReachStrategy::NO_STRATEGY;
        } else if (shape >= 0 && shape <= 3) || shape == 9 {
            return ReachStrategy::WALL_STRATEGY;
        } else if shape < 9 {
            return ReachStrategy::WALL_DECOR_STRATEGY;
        } else if (shape >= 10 && shape <= 11) || shape == 22 {
            return ReachStrategy::RECTANGLE_STRATEGY;
        }
        ReachStrategy::NO_STRATEGY
    }

    #[inline(always)]
    pub(crate) const fn altered_rotation(angle: u8, shape: i8) -> u8 {
        if shape == 7 { (angle + 2) & 0x3 } else { angle }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    pub(crate) unsafe fn reached(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        dest_width: u8,
        dest_height: u8,
        src_size: u8,
        angle: u8,
        shape: i8,
        block_access_flags: u8,
    ) -> bool {
        let exit_strategy: i32 = ReachStrategy::exit_strategy(shape);
        if exit_strategy != ReachStrategy::RECTANGLE_EXCLUSIVE_STRATEGY
            && src_x == dest_x
            && src_z == dest_z
        {
            return true;
        }
        match exit_strategy {
            ReachStrategy::WALL_STRATEGY => ReachStrategy::reach_wall(
                flags, y, src_x, src_z, dest_x, dest_z, src_size, shape, angle,
            ),
            ReachStrategy::WALL_DECOR_STRATEGY => ReachStrategy::reach_wall_decor(
                flags, y, src_x, src_z, dest_x, dest_z, src_size, shape, angle,
            ),
            ReachStrategy::RECTANGLE_STRATEGY => ReachStrategy::reach_rectangle(
                flags,
                y,
                src_x,
                src_z,
                dest_x,
                dest_z,
                src_size,
                dest_width,
                dest_height,
                angle,
                block_access_flags,
            ),
            ReachStrategy::RECTANGLE_EXCLUSIVE_STRATEGY => {
                ReachStrategy::reach_exclusive_rectangle(
                    flags,
                    y,
                    src_x,
                    src_z,
                    dest_x,
                    dest_z,
                    src_size,
                    dest_width,
                    dest_height,
                    angle,
                    block_access_flags,
                )
            }
            _ => false,
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_rectangle(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        dest_width: u8,
        dest_height: u8,
        angle: u8,
        block_access_flags: u8,
    ) -> bool {
        let rotated_width: u8 = rotate(angle, dest_width, dest_height);
        let rotated_height: u8 = rotate(angle, dest_height, dest_width);
        let rotated_block_access: u8 = rotate_flags(angle, block_access_flags);

        let collides: bool = collides(
            src_x,
            src_z,
            dest_x,
            dest_z,
            src_size,
            src_size,
            rotated_width,
            rotated_height,
        );

        match src_size {
            1 => {
                collides
                    || reach_rectangle_1(
                        flags,
                        y,
                        src_x,
                        src_z,
                        dest_x,
                        dest_z,
                        rotated_width,
                        rotated_height,
                        rotated_block_access,
                    )
            }
            _ => {
                collides
                    || reach_rectangle_n(
                        flags,
                        y,
                        src_x,
                        src_z,
                        dest_x,
                        dest_z,
                        src_size,
                        src_size,
                        rotated_width,
                        rotated_height,
                        rotated_block_access,
                    )
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_exclusive_rectangle(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        dest_width: u8,
        dest_height: u8,
        angle: u8,
        block_access_flags: u8,
    ) -> bool {
        let rotated_width: u8 = rotate(angle, dest_width, dest_height);
        let rotated_height: u8 = rotate(angle, dest_height, dest_width);
        let rotated_block_access: u8 = rotate_flags(angle, block_access_flags);

        let collides: bool = collides(
            src_x,
            src_z,
            dest_x,
            dest_z,
            src_size,
            src_size,
            rotated_width,
            rotated_height,
        );

        match src_size {
            1 => {
                !collides
                    && reach_rectangle_1(
                        flags,
                        y,
                        src_x,
                        src_z,
                        dest_x,
                        dest_z,
                        rotated_width,
                        rotated_height,
                        rotated_block_access,
                    )
            }
            _ => {
                !collides
                    && reach_rectangle_n(
                        flags,
                        y,
                        src_x,
                        src_z,
                        dest_x,
                        dest_z,
                        src_size,
                        src_size,
                        rotated_width,
                        rotated_height,
                        rotated_block_access,
                    )
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        shape: i8,
        angle: u8,
    ) -> bool {
        if (src_size == 1 && src_x == dest_x && src_z == dest_z)
            || (src_size != 1
                && dest_x >= src_x
                && src_size as i32 + src_x > dest_x
                && dest_z >= src_z
                && src_size as i32 + src_z > dest_z)
        {
            return true;
        } else if src_size == 1 {
            return ReachStrategy::reach_wall_1(
                flags, y, src_x, src_z, dest_x, dest_z, shape, angle,
            );
        }
        ReachStrategy::reach_wall_n(
            flags, y, src_x, src_z, dest_x, dest_z, src_size, shape, angle,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall_decor(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        shape: i8,
        angle: u8,
    ) -> bool {
        if (src_size == 1 && src_x == dest_x && src_z == dest_z)
            || (src_size != 1
                && dest_x >= src_x
                && src_size as i32 + src_x > dest_x
                && dest_z >= src_z
                && src_size as i32 + src_z > dest_z)
        {
            return true;
        } else if src_size == 1 {
            return ReachStrategy::reach_wall_decor_1(
                flags, y, src_x, src_z, dest_x, dest_z, shape, angle,
            );
        }
        ReachStrategy::reach_wall_decor_n(
            flags, y, src_x, src_z, dest_x, dest_z, src_size, shape, angle,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall_1(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        shape: i8,
        angle: u8,
    ) -> bool {
        let collision_flags: u32 = flags.get(src_x, src_z, y);
        if shape == LocShape::WallStraight {
            return match LocAngle::from(angle) {
                LocAngle::West => {
                    (src_x == dest_x - 1 && src_z == dest_z)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (src_x == dest_x && src_z == dest_z + 1)
                        || (src_x == dest_x - 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockWest as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x + 1 && src_z == dest_z)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::South => {
                    (src_x == dest_x && src_z == dest_z - 1)
                        || (src_x == dest_x - 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockWest as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                }
            };
        } else if shape == LocShape::WallL {
            return match LocAngle::from(angle) {
                LocAngle::West => {
                    (src_x == dest_x - 1 && src_z == dest_z)
                        || (src_x == dest_x && src_z == dest_z + 1)
                        || (src_x == dest_x + 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (src_x == dest_x - 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::BlockWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x && src_z == dest_z + 1)
                        || (src_x == dest_x + 1 && src_z == dest_z)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x - 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::BlockWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1 && src_z == dest_z)
                        || (src_x == dest_x && src_z == dest_z - 1)
                }
                LocAngle::South => {
                    (src_x == dest_x - 1 && src_z == dest_z)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z == dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x && src_z == dest_z - 1)
                }
            };
        } else if shape == LocShape::WallDiagonal {
            return (src_x == dest_x
                && src_z == dest_z + 1
                && (collision_flags & CollisionFlag::WallSouth as u32)
                    == CollisionFlag::Open as u32)
                || (src_x == dest_x
                    && src_z == dest_z - 1
                    && (collision_flags & CollisionFlag::WallNorth as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x - 1
                    && src_z == dest_z
                    && (collision_flags & CollisionFlag::WallEast as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x + 1
                    && src_z == dest_z
                    && (collision_flags & CollisionFlag::WallWest as u32)
                        == CollisionFlag::Open as u32);
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall_n(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        shape: i8,
        angle: u8,
    ) -> bool {
        let collision_flags: u32 = flags.get(src_x, src_z, y);
        let east: i32 = src_x + src_size as i32 - 1;
        let north: i32 = src_z + src_size as i32 - 1;
        if shape == LocShape::WallStraight {
            return match LocAngle::from(angle) {
                LocAngle::West => {
                    (src_x == dest_x - src_size as i32 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z - src_size as i32
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (dest_x >= src_x && dest_x <= east && src_z == dest_z + 1)
                        || (src_x == dest_x - src_size as i32
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockWest as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x + 1 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z - src_size as i32
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::South => {
                    (dest_x >= src_x && dest_x <= east && src_z == dest_z - src_size as i32)
                        || (src_x == dest_x - src_size as i32
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockWest as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                }
            };
        } else if shape == LocShape::WallL {
            return match LocAngle::from(angle) {
                LocAngle::West => {
                    (src_x == dest_x - src_size as i32 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x && dest_x <= east && src_z == dest_z + 1)
                        || (src_x == dest_x + 1
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z - src_size as i32
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (src_x == dest_x - src_size as i32
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::BlockWest as u32)
                            == CollisionFlag::Open as u32)
                        || (dest_x >= src_x && dest_x <= east && src_z == dest_z + 1)
                        || (src_x == dest_x + 1 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z - src_size as i32
                            && (collision_flags & CollisionFlag::BlockSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x - src_size as i32
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::BlockWest as u32)
                            == CollisionFlag::Open as u32)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x && dest_x <= east && src_z == dest_z - src_size as i32)
                }
                LocAngle::South => {
                    (src_x == dest_x - src_size as i32 && src_z <= dest_z && north >= dest_z)
                        || (dest_x >= src_x
                            && dest_x <= east
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::BlockNorth as u32)
                                == CollisionFlag::Open as u32)
                        || (src_x == dest_x + 1
                            && src_z <= dest_z
                            && north >= dest_z
                            && (collision_flags & CollisionFlag::BlockEast as u32)
                                == CollisionFlag::Open as u32)
                        || (dest_x >= src_x && dest_x <= east && src_z == dest_z - src_size as i32)
                }
            };
        } else if shape == LocShape::WallDiagonal {
            return (dest_x >= src_x
                && dest_x <= east
                && src_z == dest_z + 1
                && (collision_flags & CollisionFlag::BlockNorth as u32)
                    == CollisionFlag::Open as u32)
                || (dest_x >= src_x
                    && dest_x <= east
                    && src_z == dest_z - src_size as i32
                    && (collision_flags & CollisionFlag::BlockSouth as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x - src_size as i32
                    && src_z <= dest_z
                    && north >= dest_z
                    && (collision_flags & CollisionFlag::BlockWest as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x + 1
                    && src_z <= dest_z
                    && north >= dest_z
                    && (collision_flags & CollisionFlag::BlockEast as u32)
                        == CollisionFlag::Open as u32);
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall_decor_1(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        shape: i8,
        angle: u8,
    ) -> bool {
        let collision_flags: u32 = flags.get(src_x, src_z, y);
        if shape == LocShape::WallDecorDiagonalOffset
            || shape == LocShape::WallDecorDiagonalNoOffset
        {
            return match LocAngle::from(ReachStrategy::altered_rotation(angle, shape)) {
                LocAngle::West => {
                    (src_x == dest_x + 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::WallWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::WallNorth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (src_x == dest_x - 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::WallEast as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z - 1
                            && (collision_flags & CollisionFlag::WallNorth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x - 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::WallEast as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::WallSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::South => {
                    (src_x == dest_x + 1
                        && src_z == dest_z
                        && (collision_flags & CollisionFlag::WallWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x == dest_x
                            && src_z == dest_z + 1
                            && (collision_flags & CollisionFlag::WallSouth as u32)
                                == CollisionFlag::Open as u32)
                }
            };
        } else if shape == LocShape::WallDecorDiagonalBoth {
            return (src_x == dest_x
                && src_z == dest_z + 1
                && (collision_flags & CollisionFlag::WallSouth as u32)
                    == CollisionFlag::Open as u32)
                || (src_x == dest_x
                    && src_z == dest_z - 1
                    && (collision_flags & CollisionFlag::WallNorth as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x - 1
                    && src_z == dest_z
                    && (collision_flags & CollisionFlag::WallEast as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x + 1
                    && src_z == dest_z
                    && (collision_flags & CollisionFlag::WallWest as u32)
                        == CollisionFlag::Open as u32);
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn reach_wall_decor_n(
        flags: &CollisionFlagMap,
        y: i32,
        src_x: i32,
        src_z: i32,
        dest_x: i32,
        dest_z: i32,
        src_size: u8,
        shape: i8,
        angle: u8,
    ) -> bool {
        let collision_flags: u32 = flags.get(src_x, src_z, y);
        let east: i32 = src_x + src_size as i32 - 1;
        let north: i32 = src_z + src_size as i32 - 1;
        if shape == LocShape::WallDecorDiagonalOffset
            || shape == LocShape::WallDecorDiagonalNoOffset
        {
            return match LocAngle::from(ReachStrategy::altered_rotation(angle, shape)) {
                LocAngle::West => {
                    (src_x == dest_x + 1
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::WallWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x <= dest_x
                            && src_z == dest_z - src_size as i32
                            && east >= dest_x
                            && (collision_flags & CollisionFlag::WallNorth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::North => {
                    (src_x == dest_x - src_size as i32
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::WallEast as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x <= dest_x
                            && src_z == dest_z - src_size as i32
                            && east >= dest_x
                            && (collision_flags & CollisionFlag::WallNorth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::East => {
                    (src_x == dest_x - src_size as i32
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::WallEast as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x <= dest_x
                            && src_z == dest_z + 1
                            && east >= dest_x
                            && (collision_flags & CollisionFlag::WallSouth as u32)
                                == CollisionFlag::Open as u32)
                }
                LocAngle::South => {
                    (src_x == dest_x + 1
                        && src_z <= dest_z
                        && north >= dest_z
                        && (collision_flags & CollisionFlag::WallWest as u32)
                            == CollisionFlag::Open as u32)
                        || (src_x <= dest_x
                            && src_z == dest_z + 1
                            && east >= dest_x
                            && (collision_flags & CollisionFlag::WallSouth as u32)
                                == CollisionFlag::Open as u32)
                }
            };
        } else if shape == LocShape::WallDecorDiagonalBoth {
            return (src_x <= dest_x
                && src_z == dest_z + 1
                && east >= dest_x
                && (collision_flags & CollisionFlag::WallSouth as u32)
                    == CollisionFlag::Open as u32)
                || (src_x <= dest_x
                    && src_z == dest_z - src_size as i32
                    && east >= dest_x
                    && (collision_flags & CollisionFlag::WallNorth as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x - src_size as i32
                    && src_z <= dest_z
                    && north >= dest_z
                    && (collision_flags & CollisionFlag::WallEast as u32)
                        == CollisionFlag::Open as u32)
                || (src_x == dest_x + 1
                    && src_z <= dest_z
                    && north >= dest_z
                    && (collision_flags & CollisionFlag::WallWest as u32)
                        == CollisionFlag::Open as u32);
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::flag::block_flag::BlockAccessFlag;
    use crate::rsmod::flag::collision_flag::CollisionFlag;
    use crate::rsmod::reach::reach_strategy::ReachStrategy;

    const ARGS: [[i32; 4]; 9] = [
        [3203, 3203, 1, 1],
        [3203, 3203, 1, 2],
        [3203, 3203, 1, 3],
        [3203, 3203, 2, 1],
        [3203, 3203, 2, 2],
        [3203, 3203, 2, 3],
        [3203, 3203, 3, 1],
        [3203, 3203, 3, 2],
        [3203, 3203, 3, 3],
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
        collision
    }

    unsafe fn flag(
        flags: &mut CollisionFlagMap,
        base_x: i32,
        base_z: i32,
        width: i32,
        height: i32,
        mask: CollisionFlag,
    ) {
        for y in 0..4 {
            for z in 0..height {
                for x in 0..width {
                    flags.set(base_x + x, base_z + z, y, mask as u32);
                }
            }
        }
    }

    /**
     * Test that object rotations are taken into account within [ReachStrategy.reached]
     * and do not rely on external modifications. For example, given the parameters of
     * an object in coordinates (3203, 3203) with a dimension of 3 x 1 (width x height),
     * the following test should pass:
     *
     * Object rotation of [0] or [2]. (normal)
     * ```
     * --------
     * --------
     * --------
     * ---ooo--
     * --o   o-
     * ---ooo--
     * --------
     * --------
     * ```
     * Where:
     * - Area starts from bottom-left and makes its way to top-right. (3200,3200 - 3207,3207)
     * - ' ' (whitespace) are the tiles occupied by the rotated object.
     * - 'o' are the tiles that successfully pass [ReachStrategy.reached].
     * - '-' represents every other tile in the area. (in this case a zone, or 8x8 tile area)
     */
    #[test]
    fn test_reach_rotated_loc_normal() {
        for [obj_x, obj_z, width, height] in ARGS {
            let min_x = obj_x - 16;
            let min_z = obj_z - 16;
            let max_x = obj_x + 16;
            let max_z = obj_z + 16;

            unsafe {
                let mut collision = build_collision_map(min_x, min_z, max_x, max_z);
                flag(
                    &mut collision,
                    obj_x,
                    obj_z,
                    width,
                    height,
                    CollisionFlag::Loc,
                );

                let reached = |src_x: i32, src_z: i32, rot: u8, block_access_flags: u8| -> bool {
                    ReachStrategy::reached(
                        &collision,
                        0,
                        src_x,
                        src_z,
                        obj_x,
                        obj_z,
                        width as u8,
                        height as u8,
                        1,
                        rot,
                        -2, // Use rectangular exclusive strategy
                        block_access_flags,
                    )
                };

                for x in 0..width {
                    // Test coming from south tiles.
                    assert!(reached(obj_x + x, obj_z - 1, 0, 0));
                    assert!(reached(obj_x + x, obj_z - 1, 2, 0));
                    // Test coming from north tiles.
                    assert!(reached(obj_x + x, obj_z + height, 0, 0));
                    assert!(reached(obj_x + x, obj_z + height, 2, 0));
                    // Test coming from south tiles with access blocked.
                    assert!(!reached(
                        obj_x + x,
                        obj_z - 1,
                        0,
                        BlockAccessFlag::South as u8
                    ));
                    assert!(!reached(
                        obj_x + x,
                        obj_z - 1,
                        2,
                        BlockAccessFlag::North as u8
                    ));
                    // Test coming from north tiles with access blocked.
                    assert!(!reached(
                        obj_x + x,
                        obj_z + height,
                        0,
                        BlockAccessFlag::North as u8
                    ));
                    assert!(!reached(
                        obj_x + x,
                        obj_z + height,
                        2,
                        BlockAccessFlag::South as u8
                    ));
                }

                for z in 0..height {
                    // Test coming from west tiles.
                    assert!(reached(obj_x - 1, obj_z + z, 0, 0));
                    assert!(reached(obj_x - 1, obj_z + z, 2, 0));
                    // Test coming from east tiles.
                    assert!(reached(obj_x + width, obj_z + z, 0, 0));
                    assert!(reached(obj_x + width, obj_z + z, 2, 0));
                    // Test coming from west tiles with access blocked.
                    assert!(!reached(
                        obj_x - 1,
                        obj_z + z,
                        0,
                        BlockAccessFlag::West as u8
                    ));
                    assert!(!reached(
                        obj_x - 1,
                        obj_z + z,
                        2,
                        BlockAccessFlag::East as u8
                    ));
                    // Test coming from east tiles with access blocked.
                    assert!(!reached(
                        obj_x + width,
                        obj_z + z,
                        0,
                        BlockAccessFlag::East as u8
                    ));
                    assert!(!reached(
                        obj_x + width,
                        obj_z + z,
                        2,
                        BlockAccessFlag::West as u8
                    ));
                }
            }
        }
    }

    /**
     * Test that object rotations are taken into account within [ReachStrategy.reached]
     * and do not rely on external modifications. For example, given the parameters of
     * an object in coordinates (3203, 3203) with a dimension of 3 x 1 (width x height),
     * the following test should pass:
     *
     * Object rotation of [1] or [3]. (flipped)
     * ```
     * --------
     * ---o----
     * --o o---
     * --o o---
     * --o o---
     * ---o----
     * --------
     * --------
     * ```
     * Where:
     * - Area starts from bottom-left and makes its way to top-right. (3200,3200 - 3207,3207)
     * - ' ' (whitespace) are the tiles occupied by the rotated object.
     * - 'o' are the tiles that successfully pass [ReachStrategy.reached].
     * - '-' represents every other tile in the area. (in this case a zone, or 8x8 tile area)
     */
    #[test]
    fn test_reach_rotated_loc_flipped() {
        for [obj_x, obj_z, width, height] in ARGS {
            let min_x = obj_x - 16;
            let min_z = obj_z - 16;
            let max_x = obj_x + 16;
            let max_z = obj_z + 16;

            unsafe {
                let mut collision = build_collision_map(min_x, min_z, max_x, max_z);
                flag(
                    &mut collision,
                    obj_x,
                    obj_z,
                    width,
                    height,
                    CollisionFlag::Loc,
                );

                let reached = |src_x: i32, src_z: i32, rot: u8, block_access_flags: u8| -> bool {
                    ReachStrategy::reached(
                        &collision,
                        0,
                        src_x,
                        src_z,
                        obj_x,
                        obj_z,
                        width as u8,
                        height as u8,
                        1,
                        rot,
                        -2, // Use rectangular exclusive strategy
                        block_access_flags,
                    )
                };

                for x in 0..height {
                    // width and height are swapped
                    // Test coming from south tiles.
                    assert!(reached(obj_x + x, obj_z - 1, 1, 0));
                    assert!(reached(obj_x + x, obj_z - 1, 3, 0));
                    // Test coming from north tiles.
                    assert!(reached(obj_x + x, obj_z + width, 1, 0)); // width and height are swapped
                    assert!(reached(obj_x + x, obj_z + width, 3, 0)); // width and height are swapped

                    // Test coming from south tiles with access blocked.
                    assert!(!reached(
                        obj_x + x,
                        obj_z - 1,
                        1,
                        BlockAccessFlag::East as u8
                    ));
                    assert!(!reached(
                        obj_x + x,
                        obj_z - 1,
                        3,
                        BlockAccessFlag::West as u8
                    ));
                    // Test coming from north tiles with access blocked.
                    assert!(!reached(
                        obj_x + x,
                        obj_z + width, // width and height are swapped
                        1,
                        BlockAccessFlag::West as u8
                    ));
                    assert!(!reached(
                        obj_x + x,
                        obj_z + width, // width and height are swapped
                        3,
                        BlockAccessFlag::East as u8
                    ));
                }

                for z in 0..width {
                    // width and height are swapped
                    // Test coming from west tiles.
                    assert!(reached(obj_x - 1, obj_z + z, 1, 0));
                    assert!(reached(obj_x - 1, obj_z + z, 3, 0));
                    // Test coming from east tiles.
                    assert!(reached(obj_x + height, obj_z + z, 1, 0)); // width and height are swapped
                    assert!(reached(obj_x + height, obj_z + z, 3, 0)); // width and height are swapped

                    // Test coming from west tiles with access blocked.
                    assert!(!reached(
                        obj_x - 1,
                        obj_z + z,
                        1,
                        BlockAccessFlag::South as u8
                    ));
                    assert!(!reached(
                        obj_x - 1,
                        obj_z + z,
                        3,
                        BlockAccessFlag::North as u8
                    ));
                    // Test coming from east tiles with access blocked.
                    assert!(!reached(
                        obj_x + height, // width and height are swapped
                        obj_z + z,
                        1,
                        BlockAccessFlag::North as u8
                    ));
                    assert!(!reached(
                        obj_x + height, // width and height are swapped
                        obj_z + z,
                        3,
                        BlockAccessFlag::South as u8
                    ));
                }
            }
        }
    }

    const BLOCK_ACCESS_FLAG_TEST_ARGS: [[i32; 3]; 4] = [
        [0, 1, 1],  // north
        [1, 0, 2],  // east
        [0, -1, 4], // south
        [-1, 0, 8], // west
    ];

    const DIMENSIONS_TEST_ARGS: [[i32; 2]; 9] = [
        [1, 1],
        [1, 2],
        [1, 3],
        [2, 1],
        [2, 2],
        [2, 3],
        [3, 1],
        [3, 2],
        [3, 3],
    ];

    const WALL_STRAIGHT_STRATEGY_TEST_ARGS: [[i32; 4]; 8] = [
        [0, 0, 1, CollisionFlag::WallSouth as i32],
        [0, 0, -1, CollisionFlag::WallNorth as i32],
        [1, -1, 0, CollisionFlag::WallEast as i32],
        [1, 1, 0, CollisionFlag::WallWest as i32],
        [2, 0, 1, CollisionFlag::WallSouth as i32],
        [2, 0, -1, CollisionFlag::WallNorth as i32],
        [3, -1, 0, CollisionFlag::WallEast as i32],
        [3, 1, 0, CollisionFlag::WallWest as i32],
    ];

    const WALL_L_STRATEGY_TEST_ARGS: [[i32; 4]; 8] = [
        [0, 1, 0, CollisionFlag::WallWest as i32],
        [0, 0, -1, CollisionFlag::WallNorth as i32],
        [1, -1, 0, CollisionFlag::WallEast as i32],
        [1, 0, -1, CollisionFlag::BlockNorth as i32],
        [2, -1, 0, CollisionFlag::BlockEast as i32],
        [2, 0, 1, CollisionFlag::BlockNorth as i32],
        [3, 0, 1, CollisionFlag::BlockSouth as i32],
        [3, 1, 0, CollisionFlag::BlockWest as i32],
    ];

    const WALLDECOR_DIAGONAL_OFFSET_STRATEGY_TEST_ARGS: [[i32; 4]; 8] = [
        [0, 1, 0, CollisionFlag::WallWest as i32],
        [0, 0, -1, CollisionFlag::WallNorth as i32],
        [1, -1, 0, CollisionFlag::WallEast as i32],
        [1, 0, -1, CollisionFlag::WallNorth as i32],
        [2, -1, 0, CollisionFlag::WallEast as i32],
        [2, 0, 1, CollisionFlag::WallSouth as i32],
        [3, 1, 0, CollisionFlag::WallWest as i32],
        [3, 0, 1, CollisionFlag::WallSouth as i32],
    ];

    #[test]
    fn test_strategy_wall_decor_shape_6() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALLDECOR_DIAGONAL_OFFSET_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    6,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    6,
                    0,
                );
                assert!(reached);
            }
        }
    }

    #[test]
    fn test_strategy_wall_decor_shape_7() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALLDECOR_DIAGONAL_OFFSET_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    ReachStrategy::altered_rotation(rotation as u8, 7),
                    7,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    ReachStrategy::altered_rotation(rotation as u8, 7),
                    7,
                    0,
                );
                assert!(reached);
            }
        }
    }

    #[test]
    fn test_strategy_wall_decor_shape_8() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALLDECOR_DIAGONAL_OFFSET_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    8,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    8,
                    0,
                );
                assert!(reached);
            }
        }
    }

    #[test]
    fn test_strategy_wall_no_flags_shape_0() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_STRAIGHT_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    0,
                    0,
                );
                assert!(!reached);

                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x,
                    obj_z,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    0,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_wall_no_flags_shape_2() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_L_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    2,
                    0,
                );
                assert!(!reached);

                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x,
                    obj_z,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    2,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_wall_no_flags_shape_9() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_STRAIGHT_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    9,
                    0,
                );
                assert!(!reached);

                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    obj_x,
                    obj_z,
                    obj_x + dir_x,
                    obj_z + dir_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    9,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_wall_with_flags_shape_0() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_STRAIGHT_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    0,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    0,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_wall_with_flags_shape_2() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_L_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    2,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    2,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_wall_with_flags_shape_9() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3200;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            for [rotation, dir_x, dir_z, flag] in WALL_STRAIGHT_STRATEGY_TEST_ARGS {
                collision.set(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    9,
                    0,
                );
                assert!(!reached);

                collision.set(obj_x + dir_x, obj_z + dir_z, 0, CollisionFlag::Open as u32);
                let reached = ReachStrategy::reached(
                    &collision,
                    0,
                    src_x + dir_x,
                    obj_z + dir_z,
                    obj_x,
                    obj_z,
                    1,
                    1,
                    1,
                    rotation as u8,
                    9,
                    0,
                );
                assert!(reached);

                collision.remove(obj_x + dir_x, obj_z + dir_z, 0, flag as u32);
            }
        }
    }

    #[test]
    fn test_strategy_divided_by_loc() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3201;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            // test blocked north
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorth as u32);
            assert!(!ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free east
            collision.set(src_x, src_z, 0, CollisionFlag::WallEast as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free south
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouth as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free west
            collision.set(src_x, src_z, 0, CollisionFlag::WallWest as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free northwest
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorthWest as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free northeast
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorthEast as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free southeast
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouthEast as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));

            // test free southwest
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouthWest as u32);
            assert!(ReachStrategy::reached(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 10, 0
            ));
        }
    }

    #[test]
    fn test_strategy_divided_by_wall() {
        let src_x = 3200;
        let src_z = 3200;
        let obj_x = 3200;
        let obj_z = 3201;

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, obj_x, obj_z);

            // test blocked north
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorth as u32);
            assert!(!ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free east
            collision.set(src_x, src_z, 0, CollisionFlag::WallEast as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free south
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouth as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free west
            collision.set(src_x, src_z, 0, CollisionFlag::WallWest as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free northwest
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorthWest as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free northeast
            collision.set(src_x, src_z, 0, CollisionFlag::WallNorthEast as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free southeast
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouthEast as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));

            // test free southwest
            collision.set(src_x, src_z, 0, CollisionFlag::WallSouthWest as u32);
            assert!(ReachStrategy::reach_rectangle(
                &collision, 0, src_x, src_z, obj_x, obj_z, 1, 1, 1, 0, 0
            ));
        }
    }

    #[test]
    fn test_strategy_block_access_flag() {
        let obj_x = 3205;
        let obj_z = 3205;

        let cardinal = [[0, -1], [0, 1], [-1, 0], [1, 0]];

        for [off_x, off_z, access_flag] in BLOCK_ACCESS_FLAG_TEST_ARGS {
            unsafe {
                let mut collision = build_collision_map(obj_x, obj_z, obj_x, obj_z);
                flag(&mut collision, obj_x, obj_z, 1, 1, CollisionFlag::Loc);

                for [dx, dz] in cardinal {
                    let src_x = obj_x + dx;
                    let src_z = obj_z + dz;
                    collision.allocate_if_absent(src_x, src_z, 0);

                    let reached = ReachStrategy::reach_rectangle(
                        &collision,
                        0,
                        src_x,
                        src_z,
                        obj_x,
                        obj_z,
                        1,
                        1,
                        1,
                        0,
                        access_flag as u8,
                    );

                    if dx == off_x && dz == off_z {
                        assert!(!reached);
                    } else {
                        assert!(reached);
                    }
                }
            }
        }
    }

    #[test]
    fn test_strategy_block_access_flag_exclusive() {
        let obj_x = 3205;
        let obj_z = 3205;

        let cardinal = [[0, -1], [0, 1], [-1, 0], [1, 0]];

        for [off_x, off_z, access_flag] in BLOCK_ACCESS_FLAG_TEST_ARGS {
            unsafe {
                let mut collision = build_collision_map(obj_x, obj_z, obj_x, obj_z);
                flag(&mut collision, obj_x, obj_z, 1, 1, CollisionFlag::Loc);

                for [dx, dz] in cardinal {
                    let src_x = obj_x + dx;
                    let src_z = obj_z + dz;
                    collision.allocate_if_absent(src_x, src_z, 0);

                    let reached = ReachStrategy::reach_exclusive_rectangle(
                        &collision,
                        0,
                        src_x,
                        src_z,
                        obj_x,
                        obj_z,
                        1,
                        1,
                        1,
                        0,
                        access_flag as u8,
                    );

                    if dx == off_x && dz == off_z {
                        assert!(!reached);
                    } else {
                        assert!(reached);
                    }
                }
            }
        }
    }

    #[test]
    fn test_strategy_reach_with_dimensions() {
        for [width, height] in DIMENSIONS_TEST_ARGS {
            let obj_x = 3202 + width;
            let obj_z = 3205;

            unsafe {
                let mut collision =
                    build_collision_map(obj_x - 1, obj_z - 1, obj_x + width, obj_z + height);
                flag(
                    &mut collision,
                    obj_x,
                    obj_z,
                    width,
                    height,
                    CollisionFlag::Loc,
                );

                let reached1 = ReachStrategy::reach_rectangle(
                    &collision,
                    0,
                    obj_x - 2,
                    obj_z - 1,
                    obj_x,
                    obj_z,
                    1,
                    width as u8,
                    height as u8,
                    0,
                    0,
                );
                assert!(!reached1);

                let reached2 = ReachStrategy::reach_rectangle(
                    &collision,
                    0,
                    obj_x - 1,
                    obj_z - 2,
                    obj_x,
                    obj_z,
                    1,
                    width as u8,
                    height as u8,
                    0,
                    0,
                );
                assert!(!reached2);

                for x in -1..width + 1 {
                    for z in -1..height + 1 {
                        let reached3 = ReachStrategy::reach_rectangle(
                            &collision,
                            0,
                            obj_x + x,
                            obj_z + z,
                            obj_x,
                            obj_z,
                            1,
                            width as u8,
                            height as u8,
                            0,
                            0,
                        );
                        let diagonal = (z == -1 && x == -1)
                            || (z == height && x == width)
                            || (z == -1 && x == width)
                            || (z == height && x == -1);
                        if diagonal {
                            assert!(!reached3);
                            continue;
                        }
                        assert!(reached3);
                    }
                }
            }
        }
    }

    #[test]
    fn test_strategy_reach_with_dimensions_exclusive() {
        for [width, height] in DIMENSIONS_TEST_ARGS {
            let obj_x = 3202 + width;
            let obj_z = 3205;

            unsafe {
                let mut collision =
                    build_collision_map(obj_x - 1, obj_z - 1, obj_x + width, obj_z + height);

                flag(
                    &mut collision,
                    obj_x,
                    obj_z,
                    width,
                    height,
                    CollisionFlag::Loc,
                );

                let reached1 = ReachStrategy::reach_exclusive_rectangle(
                    &collision,
                    0,
                    obj_x - 2,
                    obj_z - 1,
                    obj_x,
                    obj_z,
                    1,
                    width as u8,
                    height as u8,
                    0,
                    0,
                );
                assert!(!reached1);

                let reached2 = ReachStrategy::reach_exclusive_rectangle(
                    &collision,
                    0,
                    obj_x - 1,
                    obj_z - 2,
                    obj_x,
                    obj_z,
                    1,
                    width as u8,
                    height as u8,
                    0,
                    0,
                );
                assert!(!reached2);

                for x in -1..width + 1 {
                    for z in -1..height + 1 {
                        let reached3 = ReachStrategy::reach_exclusive_rectangle(
                            &collision,
                            0,
                            obj_x + x,
                            obj_z + z,
                            obj_x,
                            obj_z,
                            1,
                            width as u8,
                            height as u8,
                            0,
                            0,
                        );
                        let diagonal = (z == -1 && x == -1)
                            || (z == height && x == width)
                            || (z == -1 && x == width)
                            || (z == height && x == -1);
                        if diagonal {
                            assert!(!reached3);
                            continue;
                        }
                        let in_loc_area = 0 <= x && width > x && 0 <= z && height > z;
                        if in_loc_area {
                            assert!(!reached3);
                            continue;
                        }
                        assert!(reached3);
                    }
                }
            }
        }
    }
}
