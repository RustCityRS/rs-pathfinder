#![allow(non_camel_case_types)]

use std::cmp::PartialEq;
use std::process;

#[repr(i8)]
#[derive(Clone, Copy, PartialEq)]
pub(crate) enum LocShape {
    WallStraight = 0,
    WallDiagonalCorner = 1,
    WallL = 2,
    WallSquareCorner = 3,
    WallDecorStraightNoOffset = 4,
    WallDecorStraightOffset = 5,
    WallDecorDiagonalOffset = 6,
    WallDecorDiagonalNoOffset = 7,
    WallDecorDiagonalBoth = 8,
    WallDiagonal = 9,
    CentrepieceStraight = 10,
    CentrepieceDiagonal = 11,
    RoofStraight = 12,
    RoofDiagonalWithRoofEdge = 13,
    RoofDiagonal = 14,
    RoofLConcave = 15,
    RoofLConvex = 16,
    RoofFlat = 17,
    RoofEdgeStraight = 18,
    RoofEdgeDiagonalCorner = 19,
    RoofEdgeL = 20,
    RoofEdgeSquareCorner = 21,
    GroundDecor = 22,
}

impl From<i8> for LocShape {
    #[inline(always)]
    fn from(value: i8) -> LocShape {
        match value {
            0 => LocShape::WallStraight,
            1 => LocShape::WallDiagonalCorner,
            2 => LocShape::WallL,
            3 => LocShape::WallSquareCorner,
            4 => LocShape::WallDecorStraightNoOffset,
            5 => LocShape::WallDecorStraightOffset,
            6 => LocShape::WallDecorDiagonalOffset,
            7 => LocShape::WallDecorDiagonalNoOffset,
            8 => LocShape::WallDecorDiagonalBoth,
            9 => LocShape::WallDiagonal,
            10 => LocShape::CentrepieceStraight,
            11 => LocShape::CentrepieceDiagonal,
            12 => LocShape::RoofStraight,
            13 => LocShape::RoofDiagonalWithRoofEdge,
            14 => LocShape::RoofDiagonal,
            15 => LocShape::RoofLConcave,
            16 => LocShape::RoofLConvex,
            17 => LocShape::RoofFlat,
            18 => LocShape::RoofEdgeStraight,
            19 => LocShape::RoofEdgeDiagonalCorner,
            20 => LocShape::RoofEdgeL,
            21 => LocShape::RoofEdgeSquareCorner,
            22 => LocShape::GroundDecor,
            _ => process::abort(), // unreachable!("[LocShape] Invalid value used for shape! {}", value),
        }
    }
}

impl PartialEq<LocShape> for i8 {
    #[inline(always)]
    fn eq(&self, other: &LocShape) -> bool {
        *self == *other as i8
    }
}
