#![allow(non_camel_case_types)]

use std::process;

#[repr(u8)]
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum LocAngle {
    West = 0,
    North = 1,
    East = 2,
    South = 3,
}

impl From<u8> for LocAngle {
    #[inline(always)]
    fn from(value: u8) -> LocAngle {
        match value {
            0 => LocAngle::West,
            1 => LocAngle::North,
            2 => LocAngle::East,
            3 => LocAngle::South,
            _ => process::abort(), //unreachable!("[LocAngle] Invalid value used for angle! {}", value),
        }
    }
}

impl PartialEq<LocAngle> for u8 {
    #[inline(always)]
    fn eq(&self, other: &LocAngle) -> bool {
        *self == *other as u8
    }
}
