use crate::rsmod::flag::collision_flag::CollisionFlag;

#[derive(Clone)]
pub(crate) struct CollisionFlagMap {
    pub flags: Vec<Option<Box<[u32; 8 * 8]>>>,
}

impl CollisionFlagMap {
    const ZONE_TILE_COUNT: usize = 8 * 8;
    const TOTAL_ZONE_COUNT: usize = 256 * 256 * 4 * CollisionFlagMap::ZONE_TILE_COUNT;

    #[inline(always)]
    pub const fn zone_index(x: i32, z: i32, y: i32) -> usize {
        (((x >> 3) & 0x7ff) | (((z >> 3) & 0x7ff) << 11) | ((y & 0x3) << 22)) as usize
    }

    #[inline(always)]
    pub const fn tile_index(x: i32, z: i32) -> usize {
        ((x & 0x7) | ((z & 0x7) << 3)) as usize
    }

    #[inline(always)]
    pub fn new() -> CollisionFlagMap {
        CollisionFlagMap {
            flags: vec![None; CollisionFlagMap::TOTAL_ZONE_COUNT],
        }
    }

    #[inline(always)]
    pub unsafe fn get(&self, x: i32, z: i32, y: i32) -> u32 {
        if let Some(ref flags) = *self
            .flags
            .as_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y))
        {
            return *flags.as_ptr().add(CollisionFlagMap::tile_index(x, z));
        }
        CollisionFlag::Null as u32
    }

    #[inline(always)]
    pub unsafe fn set(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        *self
            .allocate_if_absent_return(CollisionFlagMap::zone_index(x, z, y))
            .as_mut_ptr()
            .add(CollisionFlagMap::tile_index(x, z)) = mask;
    }

    #[inline(always)]
    pub unsafe fn add(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        *self
            .allocate_if_absent_return(CollisionFlagMap::zone_index(x, z, y))
            .as_mut_ptr()
            .add(CollisionFlagMap::tile_index(x, z)) |= mask;
    }

    #[inline(always)]
    pub unsafe fn remove(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        *self
            .allocate_if_absent_return(CollisionFlagMap::zone_index(x, z, y))
            .as_mut_ptr()
            .add(CollisionFlagMap::tile_index(x, z)) &= !mask;
    }

    #[inline(always)]
    pub unsafe fn allocate_if_absent(&mut self, x: i32, z: i32, y: i32) {
        self.allocate_if_absent_return(CollisionFlagMap::zone_index(x, z, y));
    }

    #[inline(always)]
    unsafe fn allocate_if_absent_return(&mut self, zone_idx: usize) -> &mut [u32; 64] {
        (*self.flags.as_mut_ptr().add(zone_idx)).get_or_insert_with(|| {
            Box::new([CollisionFlag::Open as u32; CollisionFlagMap::ZONE_TILE_COUNT])
        })
    }

    #[inline(always)]
    pub unsafe fn deallocate_if_present(&mut self, x: i32, z: i32, y: i32) {
        *self
            .flags
            .as_mut_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y)) = None;
    }

    #[inline(always)]
    pub unsafe fn is_zone_allocated(&self, x: i32, z: i32, y: i32) -> bool {
        (*self
            .flags
            .as_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y)))
        .is_some()
    }

    #[inline(always)]
    pub unsafe fn is_flagged(&self, x: i32, z: i32, y: i32, masks: u32) -> bool {
        match &*self
            .flags
            .as_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y))
        {
            None => false,
            Some(flags) => {
                *flags.as_ptr().add(CollisionFlagMap::tile_index(x, z)) & masks
                    != CollisionFlag::Open as u32
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionFlagMap;
    use crate::rsmod::flag::collision_flag::CollisionFlag;

    #[test]
    fn test_collision_get_collision_flag_null_zone() {
        let collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe { assert_eq!(false, collision.is_zone_allocated(3200, 3200, 0)) }

        for x in 3200..3208 {
            for z in 3200..3208 {
                unsafe {
                    assert_eq!(CollisionFlag::Null as u32, collision.get(x, z, 0));
                }
            }
        }
    }

    #[test]
    fn test_collision_get_collision_flag_allocated_zone() {
        let mut collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe {
            assert_eq!(false, collision.is_zone_allocated(3200, 3200, 0));

            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(true, collision.is_zone_allocated(3200, 3200, 0));
        }

        for x in 3200..3208 {
            for z in 3200..3208 {
                unsafe {
                    assert_eq!(CollisionFlag::Open as u32, collision.get(x, z, 0));
                }
            }
        }
    }

    #[test]
    fn test_collision_set_collision_flag() {
        let mut collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe {
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 0));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 1));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 2));

            collision.set(3200, 3200, 0, CollisionFlag::Loc as u32);
            collision.set(3200, 3200, 1, CollisionFlag::Floor as u32);
            collision.set(3200, 3200, 2, CollisionFlag::Open as u32);

            assert_eq!(CollisionFlag::Loc as u32, collision.get(3200, 3200, 0));
            assert_eq!(CollisionFlag::Floor as u32, collision.get(3200, 3200, 1));
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 2));
        }
    }

    #[test]
    fn test_collision_add_collision_flag() {
        let mut collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe {
            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));

            collision.add(3200, 3200, 0, CollisionFlag::WallEastProjBlocker as u32);
            assert_eq!(
                CollisionFlag::WallEastProjBlocker as u32,
                collision.get(3200, 3200, 0)
            );

            collision.add(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert_eq!(
                true,
                collision.is_flagged(3200, 3200, 0, CollisionFlag::WallEastProjBlocker as u32)
            );
            assert_eq!(
                true,
                collision.is_flagged(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32)
            );
        }

        for x in 3201..3208 {
            for z in 3201..3208 {
                unsafe {
                    assert_eq!(CollisionFlag::Open as u32, collision.get(x, z, 0));
                }
            }
        }
    }

    #[test]
    fn test_collision_remove_collision_flag() {
        let mut collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe {
            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));

            collision.add(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert_eq!(
                CollisionFlag::WallNorthProjBlocker as u32,
                collision.get(3200, 3200, 0)
            );

            collision.remove(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));
        }
    }

    #[test]
    fn test_collision_deallocate_if_present() {
        let mut collision: CollisionFlagMap = CollisionFlagMap::new();

        unsafe {
            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));
        }

        unsafe {
            collision.deallocate_if_present(3200, 3200, 0);
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 0));
        }
    }

    #[test]
    fn test_collision_zone_index() {
        assert_eq!(CollisionFlagMap::zone_index(0, 0, 0), 0);
    }
}
