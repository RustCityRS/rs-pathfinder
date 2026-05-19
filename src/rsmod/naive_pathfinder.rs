use rand::RngExt;

use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::CollisionStrategy;
use crate::rsmod::coord_grid::CoordGrid;
use crate::rsmod::step_validator::can_travel;

const DIRECTIONS: [[i32; 2]; 4] = [
    [-1, 0], // West
    [1, 0],  // East
    [0, 1],  // North
    [0, -1], // South
];

static mut RESULT: [u32; 1] = [0; 1];

// https://gist.github.com/Z-Kris/2eb1c2fbc22aa7486a57089c82f293f8
// https://gist.github.com/Z-Kris/fe476d75a51374f12dca999700f009f7
#[allow(clippy::too_many_arguments)]
#[inline(always)]
pub(crate) unsafe fn find_naive_path(
    flags: &CollisionFlagMap,
    y: i32,
    src_x: i32,
    src_z: i32,
    dest_x: i32,
    dest_z: i32,
    src_width: u8,
    src_height: u8,
    dest_width: u8,
    dest_height: u8,
    extra_flag: u32,
    collision: &CollisionStrategy,
) -> &'static [u32] {
    // If we are intersecting at all, the path needs to try to move out of the way.
    if intersects(
        src_x,
        src_z,
        src_width,
        src_height,
        dest_x,
        dest_z,
        dest_width,
        dest_height,
    ) {
        RESULT[0] = cardinal_destination(y, src_x, src_z);
        return &RESULT;
    }
    let dest: u32 = match naive_destination(
        y,
        src_x,
        src_z,
        src_width as i32,
        src_height as i32,
        dest_x,
        dest_z,
        1,
        1,
    ) {
        Some(d) => d,
        None => return &[],
    };
    let dx: i32 = CoordGrid::from(dest).x() as i32;
    let dz: i32 = CoordGrid::from(dest).z() as i32;
    if is_diagonal(
        dx,
        dz,
        src_width as i32,
        src_height as i32,
        dest_x,
        dest_z,
        dest_width as i32,
        dest_height as i32,
    ) {
        RESULT[0] = dest;
        return &RESULT;
    }
    /* If we can interact from this coord(or overlap with the target), allow the pathfinder to exit. */
    if intersects(
        dx,
        dz,
        src_width,
        src_height,
        dest_x,
        dest_z,
        dest_width,
        dest_height,
    ) {
        RESULT[0] = dest;
        return &RESULT;
    }
    let mut curr_x: i32 = dx;
    let mut curr_z: i32 = dz;
    while curr_x != dest_x && curr_z != dest_z {
        let dx: i8 = (dest_x - curr_x).signum() as i8;
        let dz: i8 = (dest_z - curr_z).signum() as i8;
        if can_travel(
            flags, y, curr_x, curr_z, dx, dz, src_width, extra_flag, collision,
        ) {
            curr_x += dx as i32;
            curr_z += dz as i32;
        } else if dx != 0
            && can_travel(
                flags, y, curr_x, curr_z, dx, 0, src_width, extra_flag, collision,
            )
        {
            curr_x += dx as i32;
        } else if dz != 0
            && can_travel(
                flags, y, curr_x, curr_z, 0, dz, src_width, extra_flag, collision,
            )
        {
            curr_z += dz as i32;
        } else {
            /* If we can't step anywhere, exit out, we've arrived. */
            break;
        }
    }
    RESULT[0] = CoordGrid::new(y, curr_x, curr_z).0;
    &RESULT
}

/**
 * Fast way to check if two squares are intersecting.
 */
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn intersects(
    src_x: i32,
    src_z: i32,
    src_width: u8,
    src_height: u8,
    dest_x: i32,
    dest_z: i32,
    dest_width: u8,
    dest_height: u8,
) -> bool {
    let src_horizontal: i32 = src_x + src_width as i32;
    let src_vertical: i32 = src_z + src_height as i32;
    let dest_horizontal: i32 = dest_x + dest_width as i32;
    let dest_vertical: i32 = dest_z + dest_height as i32;
    !(dest_x >= src_horizontal
        || dest_horizontal <= src_x
        || dest_z >= src_vertical
        || dest_vertical <= src_z)
}

#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn is_diagonal(
    src_x: i32,
    src_z: i32,
    src_width: i32,
    src_height: i32,
    dest_x: i32,
    dest_z: i32,
    dest_width: i32,
    dest_length: i32,
) -> bool {
    if src_x + src_width == dest_x && src_z + src_height == dest_z {
        return true;
    }
    if src_x - 1 == dest_x + dest_width - 1 && src_z - 1 == dest_z + dest_length - 1 {
        return true;
    }
    if src_x + src_width == dest_x && src_z - 1 == dest_z + dest_length - 1 {
        return true;
    }
    src_x - 1 == dest_x + dest_width - 1 && src_z + src_height == dest_z
}

#[inline(always)]
unsafe fn cardinal_destination(y: i32, src_x: i32, src_z: i32) -> u32 {
    let direction: [i32; 2] = DIRECTIONS[rand::rng().random_range(0..DIRECTIONS.len())];
    CoordGrid::new(y, src_x + direction[0], src_z + direction[1]).0
}

/**
 * Calculates coordinates for [sourceX]/[sourceZ] to move to interact with [targetX]/[targetZ]
 * We first determine the cardinal direction of the source relative to the target by comparing if
 * the source lies to the left or right of diagonal \ and anti-diagonal / lines.
 * \ <= North <= /
 *  +------------+  >
 *  |            |  East
 *  +------------+  <
 * / <= South <= \
 * We then further bisect the area into three section relative to the south-west tile (zero):
 * 1. Greater than zero: follow their diagonal until the target side is reached (clamped at the furthest most tile)
 * 2. Less than zero: zero minus the size of the source
 * 3. Equal to zero: move directly towards zero / the south-west coordinate
 *
 * <  \ 0 /   <   /
 *     +---------+
 *     |         |
 *     +---------+
 * This method is equivalent to returning the last coordinate in a sequence of steps towards south-west when moving
 * ordinal then cardinally until entity side comes into contact with another.
 */
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn naive_destination(
    y: i32,
    src_x: i32,
    src_z: i32,
    src_width: i32,
    src_height: i32,
    dest_x: i32,
    dest_z: i32,
    dest_width: i32,
    dest_length: i32,
) -> Option<u32> {
    let diagonal: i32 = src_x - dest_x + (src_z - dest_z);
    let anti: i32 = src_x - dest_x - (src_z - dest_z);
    let southwest_clockwise: bool = anti < 0;
    let northwest_clockwise: bool = diagonal >= dest_length - 1 - (src_width - 1);
    let northeast_clockwise: bool = anti > src_width - src_height;
    let southeast_clockwise: bool = diagonal <= dest_width - 1 - (src_height - 1);

    if southwest_clockwise && !northwest_clockwise {
        // West
        let mut off_z: i32 = 0;
        if diagonal >= -src_width {
            off_z = coerce_at_most(diagonal + src_width, dest_length - 1);
        } else if anti > -src_width {
            off_z = -(src_width + anti);
        }
        Some(CoordGrid::new(y, -src_width + dest_x, off_z + dest_z).0)
    } else if northwest_clockwise && !northeast_clockwise {
        // North
        let mut off_x: i32 = 0;
        if anti >= -dest_length {
            off_x = coerce_at_most(anti + dest_length, dest_width - 1);
        } else if diagonal < dest_length {
            off_x = coerce_at_least(diagonal - dest_length, -(src_width - 1));
        }
        Some(CoordGrid::new(y, off_x + dest_x, dest_length + dest_z).0)
    } else if northeast_clockwise && !southeast_clockwise {
        // East
        let mut off_z: i32 = 0;
        if anti <= dest_width {
            off_z = dest_length - anti;
        } else if diagonal < dest_width {
            off_z = coerce_at_least(diagonal - dest_width, -(src_height - 1));
        }
        Some(CoordGrid::new(y, dest_width + dest_x, off_z + dest_z).0)
    } else if southeast_clockwise && !southwest_clockwise {
        // South
        let mut off_x: i32 = 0;
        if diagonal > -src_height {
            off_x = coerce_at_most(diagonal + src_height, dest_width - 1);
        } else if anti < src_height {
            off_x = coerce_at_least(anti - src_height, -(src_height - 1));
        }
        Some(CoordGrid::new(y, off_x + dest_x, -src_height + dest_z).0)
    } else {
        None
    }
}

/**
 * Ensures that this value is not greater than the specified maximumValue.
 */
#[inline(always)]
fn coerce_at_most(value: i32, max: i32) -> i32 {
    if value > max { max } else { value }
}

/**
 * Ensures that this value is not less than the specified minimumValue.
 */
#[inline(always)]
fn coerce_at_least(value: i32, min: i32) -> i32 {
    if value < min { min } else { value }
}
