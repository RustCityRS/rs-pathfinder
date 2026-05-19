use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::CollisionStrategy;
use crate::rsmod::coord_grid::CoordGrid;
use crate::rsmod::flag::collision_flag::CollisionFlag;
use crate::rsmod::flag::direction_flag::DirectionFlag;
use crate::rsmod::reach::reach_strategy::ReachStrategy;
use crate::rsmod::utils::rotation::rotate;

#[derive(Clone)]
pub(crate) struct PathFinder {
    directions: Vec<i8>,
    distances: Vec<i32>,
    generations: Vec<u32>,
    generation: u32,
    valid_local: Vec<u32>,
    curr_local_x: i32,
    curr_local_z: i32,
    buf_reader_index: usize,
    buf_writer_index: usize,
    waypoints: [u32; 25],
}

impl PathFinder {
    const DEFAULT_SEARCH_MAP_SIZE: i32 = 128;
    const DEFAULT_RING_BUFFER_SIZE: i32 = 4096;
    const DEFAULT_DISTANCE_VALUE: i32 = 99_999_999;
    const DEFAULT_SRC_DIRECTION_VALUE: i8 = 99;
    const MAX_ALTERNATIVE_ROUTE_LOWEST_COST: i32 = 1000;
    const MAX_ALTERNATIVE_ROUTE_SEEK_RANGE: i32 = 100;
    const MAX_ALTERNATIVE_ROUTE_DISTANCE_FROM_DESTINATION: i32 = 10;
    const SEARCH_HALF_MAP_SIZE: i32 = Self::DEFAULT_SEARCH_MAP_SIZE / 2;
    const MAP_SIZE: i32 = Self::DEFAULT_SEARCH_MAP_SIZE * Self::DEFAULT_SEARCH_MAP_SIZE;

    const EMPTY: &[u32] = &[];

    #[inline(always)]
    pub fn new() -> PathFinder {
        PathFinder {
            directions: vec![0; Self::MAP_SIZE as usize],
            distances: vec![Self::DEFAULT_DISTANCE_VALUE; Self::MAP_SIZE as usize],
            generations: vec![0; Self::MAP_SIZE as usize],
            generation: 1,
            valid_local: vec![0; Self::DEFAULT_RING_BUFFER_SIZE as usize],
            curr_local_x: 0,
            curr_local_z: 0,
            buf_reader_index: 0,
            buf_writer_index: 0,
            waypoints: [0; 25],
        }
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    pub unsafe fn find_path(
        &mut self,
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
        shape: i8,
        move_near: bool,
        block_access_flags: u8,
        max_waypoints: u8,
        collision: CollisionStrategy,
    ) -> &[u32] {
        self.reset();
        self.find_path_inner(
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
            shape,
            move_near,
            block_access_flags,
            max_waypoints,
            collision,
        )
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn find_path_inner(
        &mut self,
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
        shape: i8,
        move_near: bool,
        block_access_flags: u8,
        max_waypoints: u8,
        collision: CollisionStrategy,
    ) -> &[u32] {
        let base_x: i32 = src_x - Self::SEARCH_HALF_MAP_SIZE;
        let base_z: i32 = src_z - Self::SEARCH_HALF_MAP_SIZE;
        let local_src_x: i32 = src_x - base_x;
        let local_src_z: i32 = src_z - base_z;
        let local_dest_x: i32 = dest_x - base_x;
        let local_dest_z: i32 = dest_z - base_z;
        self.append_direction(
            local_src_x,
            local_src_z,
            Self::DEFAULT_SRC_DIRECTION_VALUE,
            0,
        );
        let path_found: bool = match src_size {
            1 => self.find_path_1(
                flags,
                base_x,
                base_z,
                y,
                local_dest_x,
                local_dest_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
                collision,
            ),
            2 => self.find_path_2(
                flags,
                base_x,
                base_z,
                y,
                local_dest_x,
                local_dest_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
                collision,
            ),
            _ => self.find_path_n(
                flags,
                base_x,
                base_z,
                y,
                local_dest_x,
                local_dest_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
                collision,
            ),
        };
        if !path_found {
            if !move_near {
                return Self::EMPTY;
            }
            let found_approach_point: bool = self.find_closest_approach_point(
                local_dest_x,
                local_dest_z,
                rotate(angle, dest_width, dest_height),
                rotate(angle, dest_height, dest_width),
            );
            if !found_approach_point {
                return Self::EMPTY;
            }
        }

        let limit: usize = max_waypoints as usize;

        let mut wp_len: usize = 0;

        let mut next: i8 = *self
            .directions
            .as_ptr()
            .add(Self::local_index(self.curr_local_x, self.curr_local_z));
        let mut curr: i8 = -1;

        for _ in 0..Self::MAP_SIZE {
            if self.curr_local_x == local_src_x && self.curr_local_z == local_src_z {
                break;
            }
            if curr != next {
                curr = next;
                if wp_len < limit {
                    self.waypoints[wp_len] =
                        CoordGrid::new(y, base_x + self.curr_local_x, base_z + self.curr_local_z).0;
                    wp_len += 1;
                }
            }
            if curr & DirectionFlag::East as i8 != 0 {
                self.curr_local_x += 1;
            } else if curr & DirectionFlag::West as i8 != 0 {
                self.curr_local_x -= 1;
            }
            if curr & DirectionFlag::North as i8 != 0 {
                self.curr_local_z += 1;
            } else if curr & DirectionFlag::South as i8 != 0 {
                self.curr_local_z -= 1;
            }
            next = *self
                .directions
                .as_ptr()
                .add(Self::local_index(self.curr_local_x, self.curr_local_z));
        }

        for i in 0..wp_len / 2 {
            self.waypoints.swap(i, wp_len - 1 - i);
        }

        &self.waypoints[..wp_len]
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn find_path_1(
        &mut self,
        flags: &CollisionFlagMap,
        base_x: i32,
        base_z: i32,
        y: i32,
        local_dest_x: i32,
        local_dest_z: i32,
        dest_width: u8,
        dest_height: u8,
        src_size: u8,
        angle: u8,
        shape: i8,
        block_access_flags: u8,
        collision: CollisionStrategy,
    ) -> bool {
        let mut x: i32;
        let mut z: i32;
        let mut clip_flag: CollisionFlag;
        let mut dir_flag: DirectionFlag;
        let relative_search_size: i32 = Self::DEFAULT_SEARCH_MAP_SIZE - 1;

        while self.buf_writer_index != self.buf_reader_index {
            let packed = *self.valid_local.as_ptr().add(self.buf_reader_index);
            self.curr_local_x = (packed & 0xFFFF) as i32;
            self.curr_local_z = (packed >> 16) as i32;
            self.buf_reader_index =
                (self.buf_reader_index + 1) & (Self::DEFAULT_RING_BUFFER_SIZE - 1) as usize;

            let reached: bool = ReachStrategy::reached(
                flags,
                y,
                self.curr_local_x + base_x,
                self.curr_local_z + base_z,
                local_dest_x + base_x,
                local_dest_z + base_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
            );
            if reached {
                return true;
            }

            let next_distance: i32 = *self
                .distances
                .as_ptr()
                .add(Self::local_index(self.curr_local_x, self.curr_local_z))
                + 1;

            /* east to west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z;
            clip_flag = CollisionFlag::BlockWest;
            dir_flag = DirectionFlag::East;
            if self.curr_local_x > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    clip_flag as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* west to east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z;
            clip_flag = CollisionFlag::BlockEast;
            dir_flag = DirectionFlag::West;
            if self.curr_local_x < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    clip_flag as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north to south  */
            x = self.curr_local_x;
            z = self.curr_local_z - 1;
            clip_flag = CollisionFlag::BlockSouth;
            dir_flag = DirectionFlag::North;
            if self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    clip_flag as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south to north */
            x = self.curr_local_x;
            z = self.curr_local_z + 1;
            clip_flag = CollisionFlag::BlockNorth;
            dir_flag = DirectionFlag::South;
            if self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    clip_flag as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north-east to south-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthEast;
            if self.curr_local_x > 0
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z, y),
                    CollisionFlag::BlockWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x, z, y),
                    CollisionFlag::BlockSouth as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north-west to south-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z, y),
                    CollisionFlag::BlockEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x, z, y),
                    CollisionFlag::BlockSouth as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south-east to north-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthEast;
            if self.curr_local_x > 0
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockNorthWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z, y),
                    CollisionFlag::BlockWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x, z, y),
                    CollisionFlag::BlockNorth as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south-west to north-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockNorthEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z, y),
                    CollisionFlag::BlockEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x, z, y),
                    CollisionFlag::BlockNorth as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn find_path_2(
        &mut self,
        flags: &CollisionFlagMap,
        base_x: i32,
        base_z: i32,
        y: i32,
        local_dest_x: i32,
        local_dest_z: i32,
        dest_width: u8,
        dest_height: u8,
        src_size: u8,
        angle: u8,
        shape: i8,
        block_access_flags: u8,
        collision: CollisionStrategy,
    ) -> bool {
        let mut x: i32;
        let mut z: i32;
        let mut dir_flag: DirectionFlag;
        let relative_search_size: i32 = Self::DEFAULT_SEARCH_MAP_SIZE - 2;

        while self.buf_writer_index != self.buf_reader_index {
            let packed = *self.valid_local.as_ptr().add(self.buf_reader_index);
            self.curr_local_x = (packed & 0xFFFF) as i32;
            self.curr_local_z = (packed >> 16) as i32;
            self.buf_reader_index =
                (self.buf_reader_index + 1) & (Self::DEFAULT_RING_BUFFER_SIZE - 1) as usize;

            let reached: bool = ReachStrategy::reached(
                flags,
                y,
                self.curr_local_x + base_x,
                self.curr_local_z + base_z,
                local_dest_x + base_x,
                local_dest_z + base_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
            );
            if reached {
                return true;
            }

            let next_distance: i32 = *self
                .distances
                .as_ptr()
                .add(Self::local_index(self.curr_local_x, self.curr_local_z))
                + 1;

            /* east to west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z;
            dir_flag = DirectionFlag::East;
            if self.curr_local_x > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z + 1, y),
                    CollisionFlag::BlockNorthWest as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* west to east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z;
            dir_flag = DirectionFlag::West;
            if self.curr_local_x < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x + 2, z, y),
                    CollisionFlag::BlockSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + 2,
                        self.curr_local_z + 1,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north to south  */
            x = self.curr_local_x;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::North;
            if self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x + 1, z, y),
                    CollisionFlag::BlockSouthEast as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south to north */
            x = self.curr_local_x;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::South;
            if self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z + 2, y),
                    CollisionFlag::BlockNorthWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + 1,
                        self.curr_local_z + 2,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north-east to south-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthEast;
            if self.curr_local_x > 0
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z, y),
                    CollisionFlag::BlockNorthAndSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x, z, y),
                    CollisionFlag::BlockNorthEastAndWest as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* north-west to south-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockNorthEastAndWest as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x + 2, z, y),
                    CollisionFlag::BlockSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + 2,
                        self.curr_local_z,
                        y,
                    ),
                    CollisionFlag::BlockNorthAndSouthWest as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south-east to north-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthEast;
            if self.curr_local_x > 0
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockNorthAndSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z + 2, y),
                    CollisionFlag::BlockNorthWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x,
                        self.curr_local_z + 2,
                        y,
                    ),
                    CollisionFlag::BlockSouthEastAndWest as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }

            /* south-west to north-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, self.curr_local_z + 2, y),
                    CollisionFlag::BlockSouthEastAndWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + 2,
                        self.curr_local_z + 2,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
                && collision(
                    Self::collision_flag(flags, base_x, base_z, self.curr_local_x + 2, z, y),
                    CollisionFlag::BlockNorthAndSouthWest as u32,
                )
            {
                self.append_direction(x, z, dir_flag as i8, next_distance);
            }
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    #[inline(always)]
    unsafe fn find_path_n(
        &mut self,
        flags: &CollisionFlagMap,
        base_x: i32,
        base_z: i32,
        y: i32,
        local_dest_x: i32,
        local_dest_z: i32,
        dest_width: u8,
        dest_height: u8,
        src_size: u8,
        angle: u8,
        shape: i8,
        block_access_flags: u8,
        collision: CollisionStrategy,
    ) -> bool {
        let mut x: i32;
        let mut z: i32;
        let mut dir_flag: DirectionFlag;
        let relative_search_size: i32 = Self::DEFAULT_SEARCH_MAP_SIZE - src_size as i32;

        while self.buf_writer_index != self.buf_reader_index {
            let packed = *self.valid_local.as_ptr().add(self.buf_reader_index);
            self.curr_local_x = (packed & 0xFFFF) as i32;
            self.curr_local_z = (packed >> 16) as i32;
            self.buf_reader_index =
                (self.buf_reader_index + 1) & (Self::DEFAULT_RING_BUFFER_SIZE - 1) as usize;

            let reached: bool = ReachStrategy::reached(
                flags,
                y,
                self.curr_local_x + base_x,
                self.curr_local_z + base_z,
                local_dest_x + base_x,
                local_dest_z + base_z,
                dest_width,
                dest_height,
                src_size,
                angle,
                shape,
                block_access_flags,
            );
            if reached {
                return true;
            }

            let next_distance: i32 = *self
                .distances
                .as_ptr()
                .add(Self::local_index(self.curr_local_x, self.curr_local_z))
                + 1;

            /* east to west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z;
            dir_flag = DirectionFlag::East;
            if self.curr_local_x > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        x,
                        self.curr_local_z + src_size as i32 - 1,
                        y,
                    ),
                    CollisionFlag::BlockNorthWest as u32,
                )
            {
                let clip_flag: u32 = CollisionFlag::BlockNorthAndSouthEast as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 - 1 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            x,
                            self.curr_local_z + index,
                            y,
                        ),
                        clip_flag,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* west to east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z;
            dir_flag = DirectionFlag::West;
            if self.curr_local_x < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32,
                        z,
                        y,
                    ),
                    CollisionFlag::BlockSouthEast as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32,
                        self.curr_local_z + src_size as i32 - 1,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
            {
                let clip_flag: u32 = CollisionFlag::BlockNorthAndSouthWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 - 1 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + src_size as i32,
                            self.curr_local_z + index,
                            y,
                        ),
                        clip_flag,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* north to south  */
            x = self.curr_local_x;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::North;
            if self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32 - 1,
                        z,
                        y,
                    ),
                    CollisionFlag::BlockSouthEast as u32,
                )
            {
                let clip_flag: u32 = CollisionFlag::BlockNorthEastAndWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 - 1 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + index,
                            z,
                            y,
                        ),
                        clip_flag,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* south to north */
            x = self.curr_local_x;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::South;
            if self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        x,
                        self.curr_local_z + src_size as i32,
                        y,
                    ),
                    CollisionFlag::BlockNorthWest as u32,
                )
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32 - 1,
                        self.curr_local_z + src_size as i32,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
            {
                let clip_flag: u32 = CollisionFlag::BlockSouthEastAndWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 - 1 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            x + index,
                            self.curr_local_z + src_size as i32,
                            y,
                        ),
                        clip_flag,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* north-east to south-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthEast;
            if self.curr_local_x > 0
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(flags, base_x, base_z, x, z, y),
                    CollisionFlag::BlockSouthWest as u32,
                )
            {
                let clip_flag1: u32 = CollisionFlag::BlockNorthAndSouthEast as u32;
                let clip_flag2: u32 = CollisionFlag::BlockNorthEastAndWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            x,
                            self.curr_local_z + index - 1,
                            y,
                        ),
                        clip_flag1,
                    ) || !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + index - 1,
                            z,
                            y,
                        ),
                        clip_flag2,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* north-west to south-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z - 1;
            dir_flag = DirectionFlag::NorthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z > 0
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32,
                        z,
                        y,
                    ),
                    CollisionFlag::BlockSouthEast as u32,
                )
            {
                let clip_flag1: u32 = CollisionFlag::BlockNorthAndSouthWest as u32;
                let clip_flag2: u32 = CollisionFlag::BlockNorthEastAndWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + src_size as i32,
                            self.curr_local_z + index - 1,
                            y,
                        ),
                        clip_flag1,
                    ) || !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + index,
                            z,
                            y,
                        ),
                        clip_flag2,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* south-east to north-west */
            x = self.curr_local_x - 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthEast;
            if self.curr_local_x > 0
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        x,
                        self.curr_local_z + src_size as i32,
                        y,
                    ),
                    CollisionFlag::BlockNorthWest as u32,
                )
            {
                let clip_flag1: u32 = CollisionFlag::BlockNorthAndSouthEast as u32;
                let clip_flag2: u32 = CollisionFlag::BlockSouthEastAndWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            x,
                            self.curr_local_z + index,
                            y,
                        ),
                        clip_flag1,
                    ) || !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + index - 1,
                            self.curr_local_z + src_size as i32,
                            y,
                        ),
                        clip_flag2,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }

            /* south-west to north-east */
            x = self.curr_local_x + 1;
            z = self.curr_local_z + 1;
            dir_flag = DirectionFlag::SouthWest;
            if self.curr_local_x < relative_search_size
                && self.curr_local_z < relative_search_size
                && *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                && collision(
                    Self::collision_flag(
                        flags,
                        base_x,
                        base_z,
                        self.curr_local_x + src_size as i32,
                        self.curr_local_z + src_size as i32,
                        y,
                    ),
                    CollisionFlag::BlockNorthEast as u32,
                )
            {
                let clip_flag1: u32 = CollisionFlag::BlockSouthEastAndWest as u32;
                let clip_flag2: u32 = CollisionFlag::BlockNorthAndSouthWest as u32;
                let mut blocked: bool = false;
                for index in 1..src_size as i32 {
                    if !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + index,
                            self.curr_local_z + src_size as i32,
                            y,
                        ),
                        clip_flag1,
                    ) || !collision(
                        Self::collision_flag(
                            flags,
                            base_x,
                            base_z,
                            self.curr_local_x + src_size as i32,
                            self.curr_local_z + index,
                            y,
                        ),
                        clip_flag2,
                    ) {
                        blocked = true;
                        break;
                    }
                }
                if !blocked {
                    self.append_direction(x, z, dir_flag as i8, next_distance);
                }
            }
        }
        false
    }

    #[inline(always)]
    unsafe fn find_closest_approach_point(
        &mut self,
        local_dest_x: i32,
        local_dest_z: i32,
        width: u8,
        height: u8,
    ) -> bool {
        let mut lowest_cost: i32 = Self::MAX_ALTERNATIVE_ROUTE_LOWEST_COST;
        let mut max_alternative_path: i32 = Self::MAX_ALTERNATIVE_ROUTE_SEEK_RANGE;
        let alternative_route_range: i32 = Self::MAX_ALTERNATIVE_ROUTE_DISTANCE_FROM_DESTINATION;

        for x in local_dest_x - alternative_route_range..=local_dest_x + alternative_route_range {
            for z in local_dest_z - alternative_route_range..=local_dest_z + alternative_route_range
            {
                if !((0..Self::DEFAULT_SEARCH_MAP_SIZE).contains(&x)
                    && (0..Self::DEFAULT_SEARCH_MAP_SIZE).contains(&z))
                    || *self.generations.as_ptr().add(Self::local_index(x, z)) != self.generation
                    || *self.distances.as_ptr().add(Self::local_index(x, z))
                        >= Self::MAX_ALTERNATIVE_ROUTE_SEEK_RANGE
                {
                    continue;
                }

                let mut dx: i32 = 0;
                if x < local_dest_x {
                    dx = local_dest_x - x;
                } else if x > local_dest_x + width as i32 - 1 {
                    dx = x - (width as i32 + local_dest_x - 1);
                }

                let mut dz: i32 = 0;
                if z < local_dest_z {
                    dz = local_dest_z - z;
                } else if z > local_dest_z + height as i32 - 1 {
                    dz = z - (height as i32 + local_dest_z - 1);
                }

                let cost: i32 = dx * dx + dz * dz;
                if cost < lowest_cost
                    || (cost == lowest_cost
                        && max_alternative_path
                            > *self.distances.as_ptr().add(Self::local_index(x, z)))
                {
                    self.curr_local_x = x;
                    self.curr_local_z = z;
                    lowest_cost = cost;
                    max_alternative_path = *self.distances.as_ptr().add(Self::local_index(x, z));
                }
            }
        }
        lowest_cost != Self::MAX_ALTERNATIVE_ROUTE_LOWEST_COST
    }

    #[inline(always)]
    fn local_index(x: i32, z: i32) -> usize {
        ((x << 7) | z) as usize
    }

    #[inline(always)]
    unsafe fn collision_flag(
        flags: &CollisionFlagMap,
        base_x: i32,
        base_z: i32,
        local_x: i32,
        local_z: i32,
        y: i32,
    ) -> u32 {
        flags.get(base_x + local_x, base_z + local_z, y)
    }

    #[inline(always)]
    unsafe fn append_direction(&mut self, x: i32, z: i32, direction: i8, distance: i32) {
        let index: usize = Self::local_index(x, z);
        *self.directions.as_mut_ptr().add(index) = direction;
        *self.distances.as_mut_ptr().add(index) = distance;
        *self.generations.as_mut_ptr().add(index) = self.generation;
        *self.valid_local.as_mut_ptr().add(self.buf_writer_index) = (x as u32) | ((z as u32) << 16);
        self.buf_writer_index =
            (self.buf_writer_index + 1) & (Self::DEFAULT_RING_BUFFER_SIZE - 1) as usize;
    }

    #[inline(always)]
    unsafe fn reset(&mut self) {
        self.generation = self.generation.wrapping_add(1);
        if self.generation == 0 {
            self.generation = 1;
            // perform reset every 4,294,967,295 generations.
            std::ptr::write_bytes(self.generations.as_mut_ptr(), 0, 128 * 128);
        }
        self.buf_reader_index = 0;
        self.buf_writer_index = 0;
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::collision::collision_strategy::{CollisionType, get_collision_strategy};
    use crate::rsmod::flag::collision_flag::CollisionFlag;
    use crate::rsmod::pathfinder::PathFinder;

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

    #[test]
    fn test_pf_coord_matches_level_input() {
        let src_x = 3200;
        let src_z = 3200;
        let dest_x = 3201;
        let dest_z = 3200;

        let mut pf = PathFinder::new();

        unsafe {
            let collision = build_collision_map(src_x, src_z, dest_x, dest_z);

            let mut route = pf.find_path(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            assert!(!route.is_empty());
            for index in 0..route.len() {
                assert_eq!(0, (route[index] >> 28) & 0x3);
            }

            route = pf.find_path(
                &collision,
                1,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            assert!(!route.is_empty());
            for index in 0..route.len() {
                assert_eq!(1, (route[index] >> 28) & 0x3);
            }

            route = pf.find_path(
                &collision,
                2,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            assert!(!route.is_empty());
            for index in 0..route.len() {
                assert_eq!(2, (route[index] >> 28) & 0x3);
            }

            route = pf.find_path(
                &collision,
                3,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            assert!(!route.is_empty());
            for index in 0..route.len() {
                assert_eq!(3, (route[index] >> 28) & 0x3);
            }
        }
    }

    #[test]
    fn test_pf_surrounded_by_locs_allow_move_near() {
        let src_x = 3200;
        let src_z = 3200;
        let dest_x = 3205;
        let dest_z = 3200;

        let mut pf = PathFinder::new();

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);

            flag(
                &mut collision,
                src_x - 1,
                src_z - 1,
                3,
                3,
                CollisionFlag::Loc,
            );
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32); // Remove collision flag from source tile

            let route = pf.find_path(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            // expect(route.alternative).toBeTruthy();
            assert_eq!(route.len(), 0);
        }
    }

    #[test]
    fn test_pf_surrounded_by_locs_no_move_near() {
        let src_x = 3200;
        let src_z = 3200;
        let dest_x = 3205;
        let dest_z = 3200;

        let mut pf = PathFinder::new();

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);

            flag(
                &mut collision,
                src_x - 1,
                src_z - 1,
                3,
                3,
                CollisionFlag::Loc,
            );
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32); // Remove collision flag from source tile

            let route = pf.find_path(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                false,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            // expect(route.failed).toBeTruthy();
            assert_eq!(route.len(), 0);
        }
    }

    #[test]
    fn test_pf_single_exit_point() {
        let src_x = 3200;
        let src_z = 3200;
        let dest_x = 3200;
        let dest_z = 3205;

        let mut pf = PathFinder::new();

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);

            flag(
                &mut collision,
                src_x - 1,
                src_z - 1,
                3,
                3,
                CollisionFlag::Loc,
            );
            collision.set(src_x, src_z, 0, CollisionFlag::Open as u32); // Remove collision flag from source tile
            collision.set(src_x, src_z - 1, 0, CollisionFlag::Open as u32); // Remove collision flag from tile south of source tile.

            let route = pf.find_path(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );
            // expect(route.success).toBeTruthy();
            assert_eq!(route.len(), 4);

            assert_eq!(3200, (route[0] >> 14) & 0x3fff);
            assert_eq!(3198, route[0] & 0x3fff);

            assert_eq!(3198, (route[1] >> 14) & 0x3fff);
            assert_eq!(3198, route[1] & 0x3fff);

            assert_eq!(3198, (route[2] >> 14) & 0x3fff);
            assert_eq!(3203, route[2] & 0x3fff);

            assert_eq!(dest_x as u32, (route[3] >> 14) & 0x3fff);
            assert_eq!(dest_z as u32, route[3] & 0x3fff);
        }
    }

    #[test]
    fn test_pf_standing_on_closest_approach_point() {
        let src_x = 3200;
        let src_z = 3200;
        let dest_x = 3200;
        let dest_z = 3201;

        let mut pf = PathFinder::new();

        unsafe {
            let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);

            collision.add(
                dest_x,
                dest_z,
                0,
                CollisionFlag::WallNorth as u32
                    | CollisionFlag::WallSouth as u32
                    | CollisionFlag::WallWest as u32
                    | CollisionFlag::WallEast as u32,
            );

            let route = pf.find_path(
                &collision,
                0,
                src_x,
                src_z,
                dest_x,
                dest_z,
                1,
                1,
                1,
                0,
                -1,
                true,
                0,
                25,
                get_collision_strategy(CollisionType::Normal),
            );

            // expect(route.success).toBeTruthy();
            // expect(route.alternative).toBeTruthy();
            assert_eq!(route.len(), 0);
        }
    }

    #[test]
    fn test_pf_find_path_any_size() {
        for size in 1..=3 {
            let src_x = 3200;
            let src_z = 3200;
            let dest_x = 3200;
            let dest_z = 3210 + size;

            let mut pf = PathFinder::new();

            unsafe {
                let mut collision = build_collision_map(src_x, src_z, dest_x, dest_z);

                collision.set(src_x, src_z + 1, 0, CollisionFlag::Loc as u32);

                let route = pf.find_path(
                    &collision,
                    0,
                    src_x,
                    src_z,
                    dest_x,
                    dest_z,
                    size as u8,
                    1,
                    1,
                    0,
                    -1,
                    true,
                    0,
                    25,
                    get_collision_strategy(CollisionType::Normal),
                );

                assert!(!route.is_empty());
                // expect(route.alternative).toBeFalsy();
            }
        }
    }
}
