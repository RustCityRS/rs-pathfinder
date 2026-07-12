use crate::rsmod::collision::collision::CollisionFlagMap;

pub(crate) struct ZoneFlagMap {
    flags: Box<[u8]>,
}

impl ZoneFlagMap {
    pub fn new() -> ZoneFlagMap {
        ZoneFlagMap {
            flags: vec![0u8; CollisionFlagMap::TOTAL_ZONE_COUNT].into_boxed_slice(),
        }
    }

    #[inline(always)]
    pub const unsafe fn change_zone(&mut self, x: i32, z: i32, y: i32, mask: u8, add: bool) {
        let zone = self
            .flags
            .as_mut_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y));
        if add {
            *zone |= mask;
        } else {
            *zone &= !mask;
        }
    }

    #[inline(always)]
    pub const unsafe fn is_flagged(&self, x: i32, z: i32, y: i32, mask: u8) -> bool {
        *self
            .flags
            .as_ptr()
            .add(CollisionFlagMap::zone_index(x, z, y))
            & mask
            != 0
    }

    #[inline(always)]
    pub unsafe fn borders(&self, x: i32, z: i32, y: i32, mask: u8) -> bool {
        self.is_flagged((x - 1).max(0), z, y, mask)
            || self.is_flagged(x + 1, z, y, mask)
            || self.is_flagged(x, (z - 1).max(0), y, mask)
            || self.is_flagged(x, z + 1, y, mask)
    }
}

#[cfg(test)]
mod tests {
    use crate::rsmod::collision::zone::ZoneFlagMap;
    use crate::rsmod::flag::zone_flag::ZoneFlag;

    #[test]
    fn test_zone_default_unflagged() {
        let zones: ZoneFlagMap = ZoneFlagMap::new();

        for x in 3200..3208 {
            for z in 3200..3208 {
                for y in 0..4 {
                    unsafe {
                        assert_eq!(false, zones.is_flagged(x, z, y, ZoneFlag::Free as u8));
                        assert_eq!(false, zones.is_flagged(x, z, y, ZoneFlag::Multi as u8));
                    }
                }
            }
        }
    }

    #[test]
    fn test_zone_change_round_trip() {
        let mut zones: ZoneFlagMap = ZoneFlagMap::new();

        unsafe {
            zones.change_zone(3200, 3200, 0, ZoneFlag::Free as u8, true);
            assert_eq!(true, zones.is_flagged(3200, 3200, 0, ZoneFlag::Free as u8));

            zones.change_zone(3200, 3200, 0, ZoneFlag::Free as u8, false);
            assert_eq!(false, zones.is_flagged(3200, 3200, 0, ZoneFlag::Free as u8));
        }
    }

    #[test]
    fn test_zone_flag_covers_whole_zone_not_neighbors() {
        let mut zones: ZoneFlagMap = ZoneFlagMap::new();

        unsafe {
            zones.change_zone(3204, 3204, 0, ZoneFlag::Free as u8, true);

            for x in 3200..3208 {
                for z in 3200..3208 {
                    assert_eq!(true, zones.is_flagged(x, z, 0, ZoneFlag::Free as u8));
                }
            }

            assert_eq!(false, zones.is_flagged(3199, 3204, 0, ZoneFlag::Free as u8));
            assert_eq!(false, zones.is_flagged(3208, 3204, 0, ZoneFlag::Free as u8));
            assert_eq!(false, zones.is_flagged(3204, 3199, 0, ZoneFlag::Free as u8));
            assert_eq!(false, zones.is_flagged(3204, 3208, 0, ZoneFlag::Free as u8));
            assert_eq!(false, zones.is_flagged(3204, 3204, 1, ZoneFlag::Free as u8));
        }
    }

    #[test]
    fn test_zone_masks_independent() {
        let mut zones: ZoneFlagMap = ZoneFlagMap::new();

        unsafe {
            zones.change_zone(3200, 3200, 0, ZoneFlag::Free as u8, true);
            zones.change_zone(3200, 3200, 0, ZoneFlag::Multi as u8, true);
            assert_eq!(true, zones.is_flagged(3200, 3200, 0, ZoneFlag::Free as u8));
            assert_eq!(true, zones.is_flagged(3200, 3200, 0, ZoneFlag::Multi as u8));

            zones.change_zone(3200, 3200, 0, ZoneFlag::Multi as u8, false);
            assert_eq!(true, zones.is_flagged(3200, 3200, 0, ZoneFlag::Free as u8));
            assert_eq!(
                false,
                zones.is_flagged(3200, 3200, 0, ZoneFlag::Multi as u8)
            );
        }
    }

    #[test]
    fn test_zone_borders_across_zone_boundary() {
        let mut zones: ZoneFlagMap = ZoneFlagMap::new();

        unsafe {
            zones.change_zone(3200, 3200, 0, ZoneFlag::Free as u8, true);

            assert_eq!(true, zones.borders(3208, 3200, 0, ZoneFlag::Free as u8));
            assert_eq!(true, zones.borders(3199, 3200, 0, ZoneFlag::Free as u8));
            assert_eq!(true, zones.borders(3200, 3208, 0, ZoneFlag::Free as u8));
            assert_eq!(true, zones.borders(3200, 3199, 0, ZoneFlag::Free as u8));

            assert_eq!(true, zones.borders(3204, 3204, 0, ZoneFlag::Free as u8));

            assert_eq!(false, zones.borders(3209, 3200, 0, ZoneFlag::Free as u8));
            assert_eq!(false, zones.borders(3208, 3208, 0, ZoneFlag::Free as u8));
        }
    }

    #[test]
    fn test_zone_borders_map_edge() {
        let mut zones: ZoneFlagMap = ZoneFlagMap::new();

        unsafe {
            zones.change_zone(0, 0, 0, ZoneFlag::Free as u8, true);

            assert_eq!(true, zones.borders(0, 0, 0, ZoneFlag::Free as u8));

            zones.change_zone(0, 0, 0, ZoneFlag::Free as u8, false);
            assert_eq!(false, zones.borders(0, 0, 0, ZoneFlag::Free as u8));
        }
    }
}
