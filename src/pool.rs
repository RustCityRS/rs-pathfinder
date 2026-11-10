use crate::rsmod::collision::collision::CollisionFlagMap;
use crate::rsmod::collision::collision_strategy::CollisionType;
use crate::rsmod::pathfinder::PathFinder;
use crate::{COLLISION_FLAGS, PATHFINDER, with_collision_strategy};
use rayon::prelude::*;
use rayon::{ThreadPool, ThreadPoolBuilder};
use std::cell::RefCell;
use std::io;

thread_local! {
    static FINDER: RefCell<PathFinder> = RefCell::new(PathFinder::new());
}

#[derive(Clone, Copy)]
pub struct PathRequest {
    pub y: u8,
    pub src_x: u16,
    pub src_z: u16,
    pub dest_x: u16,
    pub dest_z: u16,
    pub src_size: u8,
    pub dest_width: u8,
    pub dest_length: u8,
    pub angle: u8,
    pub shape: i8,
    pub move_near: bool,
    pub block_access_flags: u8,
    pub max_waypoints: u8,
    pub collision: CollisionType,
}

impl PathRequest {
    pub fn find_path(&self) -> &'static [u32] {
        crate::find_path(
            self.y,
            self.src_x,
            self.src_z,
            self.dest_x,
            self.dest_z,
            self.src_size,
            self.dest_width,
            self.dest_length,
            self.angle,
            self.shape,
            self.move_near,
            self.block_access_flags,
            self.max_waypoints,
            self.collision,
        )
    }
}

pub struct PathJob {
    request: PathRequest,
    waypoints: [u32; 25],
    len: usize,
}

impl PathJob {
    pub fn new(request: PathRequest) -> Self {
        Self {
            request,
            waypoints: [0; 25],
            len: 0,
        }
    }

    pub fn waypoints(&self) -> &[u32] {
        &self.waypoints[..self.len]
    }

    unsafe fn find(&mut self, finder: &mut PathFinder, flags: &CollisionFlagMap) {
        let r = self.request;
        let path = with_collision_strategy!(r.collision, strategy => finder.find_path(
            flags,
            r.y as i32,
            r.src_x as i32,
            r.src_z as i32,
            r.dest_x as i32,
            r.dest_z as i32,
            r.src_size,
            r.dest_width,
            r.dest_length,
            r.angle,
            r.shape,
            r.move_near,
            r.block_access_flags,
            r.max_waypoints.min(25),
            strategy,
        ));
        self.len = path.len();
        self.waypoints[..self.len].copy_from_slice(path);
    }
}

pub struct PathPool {
    pool: ThreadPool,
}

impl PathPool {
    /// Creates the workers before the engine raises its thread priority.
    pub fn new(threads: usize) -> io::Result<Self> {
        if threads == 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "zero pathfinder threads",
            ));
        }
        let pool = ThreadPoolBuilder::new()
            .num_threads(threads)
            .thread_name(|i| format!("pathfinder-{i}"))
            .build()
            .map_err(io::Error::other)?;
        Ok(Self { pool })
    }

    /// Completes every search before returning, keeping each result in its job.
    ///
    /// # Safety
    /// Call only from the collision map's owning thread, with no concurrent
    /// writes to the map. The caller must also exclude synchronous searches
    /// on other threads, since small batches reuse the synchronous finder.
    pub unsafe fn find_paths(&self, jobs: &mut [PathJob]) {
        if jobs.is_empty() {
            return;
        }
        let flags = &*COLLISION_FLAGS;
        if jobs.len() < self.pool.current_num_threads() * 4 {
            for job in jobs {
                job.find(&mut PATHFINDER, flags);
            }
            return;
        }
        self.pool.install(|| {
            jobs.par_chunks_mut(8).for_each(|chunk| {
                FINDER.with_borrow_mut(|finder| {
                    for job in chunk {
                        job.find(finder, flags);
                    }
                });
            });
        });
    }
}
