use std::ops::BitAnd;

#[repr(u8)]
pub(crate) enum DirectionFlag {
    North = 0x1,
    East = 0x2,
    South = 0x4,
    West = 0x8,

    SouthWest = 0x8 | 0x4,
    NorthWest = 0x8 | 0x1,
    SouthEast = 0x2 | 0x4,
    NorthEast = 0x2 | 0x1,
}

impl BitAnd<DirectionFlag> for i8 {
    type Output = i8;

    #[inline(always)]
    fn bitand(self, rhs: DirectionFlag) -> Self::Output {
        self & rhs as i8
    }
}
