#[inline(always)]
pub(crate) fn rotate(angle: u8, a: u8, b: u8) -> u8 {
    let mask = (angle & 1).wrapping_neg(); // 0x00 or 0xFF
    (a & !mask) | (b & mask)
}

#[inline(always)]
pub(crate) fn rotate_flags(angle: u8, block_access_flags: u8) -> u8 {
    ((block_access_flags << angle) & 0xf) | (block_access_flags >> (4u8.wrapping_sub(angle) & 0x3))
}

#[cfg(test)]
mod tests {
    use crate::rsmod::flag::block_flag::BlockAccessFlag;
    use crate::rsmod::utils::rotation::{rotate, rotate_flags};

    #[test]
    fn test_rotate_width() {
        let width = 3;
        let height = 2;
        assert_eq!(width, rotate(0, width, height));
        assert_eq!(height, rotate(1, width, height));
        assert_eq!(width, rotate(2, width, height));
        assert_eq!(height, rotate(3, width, height));
    }

    #[test]
    fn test_rotate_height() {
        let width = 3;
        let height = 2;
        assert_eq!(height, rotate(0, height, width));
        assert_eq!(width, rotate(1, height, width));
        assert_eq!(height, rotate(2, height, width));
        assert_eq!(width, rotate(3, height, width));
    }

    #[test]
    fn test_rotate_block_access_flag_north() {
        let north = BlockAccessFlag::North as u8;
        assert_eq!(BlockAccessFlag::North as u8, rotate_flags(0, north));
        assert_eq!(BlockAccessFlag::East as u8, rotate_flags(1, north));
        assert_eq!(BlockAccessFlag::South as u8, rotate_flags(2, north));
        assert_eq!(BlockAccessFlag::West as u8, rotate_flags(3, north));
    }

    #[test]
    fn test_rotate_block_access_flag_east() {
        let east = BlockAccessFlag::East as u8;
        assert_eq!(BlockAccessFlag::East as u8, rotate_flags(0, east));
        assert_eq!(BlockAccessFlag::South as u8, rotate_flags(1, east));
        assert_eq!(BlockAccessFlag::West as u8, rotate_flags(2, east));
        assert_eq!(BlockAccessFlag::North as u8, rotate_flags(3, east));
    }

    #[test]
    fn test_rotate_block_access_flag_south() {
        let south = BlockAccessFlag::South as u8;
        assert_eq!(BlockAccessFlag::South as u8, rotate_flags(0, south));
        assert_eq!(BlockAccessFlag::West as u8, rotate_flags(1, south));
        assert_eq!(BlockAccessFlag::North as u8, rotate_flags(2, south));
        assert_eq!(BlockAccessFlag::East as u8, rotate_flags(3, south));
    }

    #[test]
    fn test_rotate_block_access_flag_west() {
        let west = BlockAccessFlag::West as u8;
        assert_eq!(BlockAccessFlag::West as u8, rotate_flags(0, west));
        assert_eq!(BlockAccessFlag::North as u8, rotate_flags(1, west));
        assert_eq!(BlockAccessFlag::East as u8, rotate_flags(2, west));
        assert_eq!(BlockAccessFlag::South as u8, rotate_flags(3, west));
    }
}
