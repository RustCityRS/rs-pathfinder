use crate::rsmod::flag::collision_flag::CollisionFlag;

#[derive(Clone)]
pub(crate) struct CollisionMap {
    pub flags: Vec<Option<Box<[u32; 8 * 8]>>>,
}

impl CollisionMap {
    const ZONE_TILE_COUNT: usize = 8 * 8;
    pub(crate) const TOTAL_ZONE_COUNT: usize = 256 * 256 * 4 * CollisionMap::ZONE_TILE_COUNT;

    #[inline(always)]
    pub const fn zone_index(x: i32, z: i32, y: i32) -> usize {
        (((x >> 3) & 0x7ff) | (((z >> 3) & 0x7ff) << 11) | ((y & 0x3) << 22)) as usize
    }

    #[inline(always)]
    pub const fn tile_index(x: i32, z: i32) -> usize {
        ((x & 0x7) | ((z & 0x7) << 3)) as usize
    }

    #[inline(always)]
    pub fn new() -> CollisionMap {
        CollisionMap {
            flags: vec![None; CollisionMap::TOTAL_ZONE_COUNT],
        }
    }

    #[inline(always)]
    pub const unsafe fn get(&self, x: i32, z: i32, y: i32) -> u32 {
        if let Some(ref flags) = *self.flags.as_ptr().add(CollisionMap::zone_index(x, z, y)) {
            return *flags.as_ptr().add(CollisionMap::tile_index(x, z));
        }
        CollisionFlag::Null as u32
    }

    #[inline(always)]
    pub unsafe fn set(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        *self
            .allocate_if_absent_return(CollisionMap::zone_index(x, z, y))
            .as_mut_ptr()
            .add(CollisionMap::tile_index(x, z)) = mask;
    }

    /// Sets `mask` on an allocated zone's tile. An absent zone stays absent,
    /// as in [`remove`](Self::remove): only [`allocate_if_absent`](Self::allocate_if_absent)
    /// and [`set`](Self::set) make a zone.
    #[inline(always)]
    pub unsafe fn add_cmap(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        if let Some(flags) = &mut *self
            .flags
            .as_mut_ptr()
            .add(CollisionMap::zone_index(x, z, y))
        {
            *flags.as_mut_ptr().add(CollisionMap::tile_index(x, z)) |= mask;
        }
    }

    /// Clears `mask` on an allocated zone's tile. An absent zone stays absent:
    /// allocating it here would turn the whole zone from `Null` into walkable
    /// `Open` floor.
    #[inline(always)]
    pub unsafe fn rem_cmap(&mut self, x: i32, z: i32, y: i32, mask: u32) {
        if let Some(flags) = &mut *self
            .flags
            .as_mut_ptr()
            .add(CollisionMap::zone_index(x, z, y))
        {
            *flags.as_mut_ptr().add(CollisionMap::tile_index(x, z)) &= !mask;
        }
    }

    /// Hints the CPU to fetch the zone table entry for (x, z, y).
    #[cfg_attr(not(target_arch = "x86_64"), allow(unused_variables))]
    #[inline(always)]
    pub unsafe fn prefetch_zone(&self, x: i32, z: i32, y: i32) {
        #[cfg(target_arch = "x86_64")]
        core::arch::x86_64::_mm_prefetch::<{ core::arch::x86_64::_MM_HINT_T0 }>(
            self.flags.as_ptr().add(CollisionMap::zone_index(x, z, y)) as *const i8,
        );
    }

    /// Hints the CPU to fetch the tile flags for (x, z, y). This reads the zone table entry,
    /// so it should follow a [`Self::prefetch_zone`] for the same tile.
    #[cfg_attr(not(target_arch = "x86_64"), allow(unused_variables))]
    #[inline(always)]
    pub unsafe fn prefetch_tile(&self, x: i32, z: i32, y: i32) {
        #[cfg(target_arch = "x86_64")]
        if let Some(ref flags) = *self.flags.as_ptr().add(CollisionMap::zone_index(x, z, y)) {
            core::arch::x86_64::_mm_prefetch::<{ core::arch::x86_64::_MM_HINT_T0 }>(
                flags.as_ptr().add(CollisionMap::tile_index(x, z)) as *const i8,
            );
        }
    }

    #[inline(always)]
    pub unsafe fn allocate_if_absent(&mut self, x: i32, z: i32, y: i32) {
        self.allocate_if_absent_return(CollisionMap::zone_index(x, z, y));
    }

    #[inline(always)]
    unsafe fn allocate_if_absent_return(&mut self, zone_idx: usize) -> &mut [u32; 64] {
        (*self.flags.as_mut_ptr().add(zone_idx)).get_or_insert_with(|| {
            Box::new([CollisionFlag::Open as u32; CollisionMap::ZONE_TILE_COUNT])
        })
    }

    #[inline(always)]
    pub unsafe fn deallocate_if_present(&mut self, x: i32, z: i32, y: i32) {
        *self
            .flags
            .as_mut_ptr()
            .add(CollisionMap::zone_index(x, z, y)) = None;
    }

    #[inline(always)]
    pub const unsafe fn is_zone_allocated(&self, x: i32, z: i32, y: i32) -> bool {
        (*self.flags.as_ptr().add(CollisionMap::zone_index(x, z, y))).is_some()
    }

    #[inline(always)]
    pub const unsafe fn is_flagged(&self, x: i32, z: i32, y: i32, masks: u32) -> bool {
        match &*self.flags.as_ptr().add(CollisionMap::zone_index(x, z, y)) {
            None => false,
            Some(flags) => {
                *flags.as_ptr().add(CollisionMap::tile_index(x, z)) & masks
                    != CollisionFlag::Open as u32
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::collision::CollisionMap;
    use crate::rsmod::flag::collision_flag::CollisionFlag;

    #[test]
    fn test_collision_get_collision_flag_null_zone() {
        let collision: CollisionMap = CollisionMap::new();

        unsafe { assert!(!collision.is_zone_allocated(3200, 3200, 0)) }

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
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            assert!(!collision.is_zone_allocated(3200, 3200, 0));

            collision.allocate_if_absent(3200, 3200, 0);
            assert!(collision.is_zone_allocated(3200, 3200, 0));
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
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 0));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 1));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 2));

            collision.set(3200, 3200, 0, CollisionFlag::Loc as u32);
            collision.set(3200, 3200, 1, CollisionFlag::Ground as u32);
            collision.set(3200, 3200, 2, CollisionFlag::Open as u32);

            assert_eq!(CollisionFlag::Loc as u32, collision.get(3200, 3200, 0));
            assert_eq!(CollisionFlag::Ground as u32, collision.get(3200, 3200, 1));
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 2));
        }
    }

    #[test]
    fn test_collision_add_collision_flag() {
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));

            collision.add_cmap(3200, 3200, 0, CollisionFlag::WallEastProjBlocker as u32);
            assert_eq!(
                CollisionFlag::WallEastProjBlocker as u32,
                collision.get(3200, 3200, 0)
            );

            collision.add_cmap(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert!(collision.is_flagged(3200, 3200, 0, CollisionFlag::WallEastProjBlocker as u32));
            assert!(collision.is_flagged(
                3200,
                3200,
                0,
                CollisionFlag::WallNorthProjBlocker as u32
            ));
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
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            collision.allocate_if_absent(3200, 3200, 0);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));

            collision.add_cmap(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert_eq!(
                CollisionFlag::WallNorthProjBlocker as u32,
                collision.get(3200, 3200, 0)
            );

            collision.rem_cmap(3200, 3200, 0, CollisionFlag::WallNorthProjBlocker as u32);
            assert_eq!(CollisionFlag::Open as u32, collision.get(3200, 3200, 0));
        }
    }

    #[test]
    fn test_collision_add_leaves_absent_zone_absent() {
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            collision.add_cmap(3200, 3200, 0, CollisionFlag::WallEast as u32);
            assert!(!collision.is_zone_allocated(3200, 3200, 0));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3207, 3207, 0));
        }
    }

    #[test]
    fn test_collision_remove_leaves_absent_zone_absent() {
        let mut collision: CollisionMap = CollisionMap::new();

        unsafe {
            collision.rem_cmap(3200, 3200, 0, CollisionFlag::Npc as u32);
            assert!(!collision.is_zone_allocated(3200, 3200, 0));
            assert_eq!(CollisionFlag::Null as u32, collision.get(3200, 3200, 0));
        }
    }

    #[test]
    fn test_collision_deallocate_if_present() {
        let mut collision: CollisionMap = CollisionMap::new();

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
        assert_eq!(CollisionMap::zone_index(0, 0, 0), 0);
    }
}
