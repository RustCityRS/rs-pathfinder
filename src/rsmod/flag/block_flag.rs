#![allow(non_camel_case_types)]

use std::ops::BitAnd;

#[repr(u8)]
#[derive(Debug)]
pub(crate) enum BlockAccessFlag {
    North = 0x1,
    East = 0x2,
    South = 0x4,
    West = 0x8,
}

impl BitAnd<BlockAccessFlag> for u8 {
    type Output = u8;

    #[inline(always)]
    fn bitand(self, rhs: BlockAccessFlag) -> Self::Output {
        self & rhs as u8
    }
}
