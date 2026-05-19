pub struct CoordGrid(pub u32);

impl CoordGrid {
    #[inline(always)]
    pub const fn new(y: i32, x: i32, z: i32) -> CoordGrid {
        CoordGrid(((z & 0x3fff) | ((x & 0x3fff) << 14) | ((y & 0x3) << 28)) as u32)
    }

    #[inline(always)]
    pub const fn from(packed: u32) -> CoordGrid {
        CoordGrid(packed)
    }

    #[inline(always)]
    pub const fn y(&self) -> u32 {
        (self.0 >> 28) & 0x3
    }

    #[inline(always)]
    pub const fn x(&self) -> u32 {
        (self.0 >> 14) & 0x3fff
    }

    #[inline(always)]
    pub const fn z(&self) -> u32 {
        self.0 & 0x3fff
    }
}
