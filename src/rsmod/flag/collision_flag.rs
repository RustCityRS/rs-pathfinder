#![allow(non_camel_case_types)]

#[repr(u32)]
#[derive(Clone, PartialEq, Copy, Debug)]
pub enum CollisionFlag {
    Open = 0x0,
    WallNorthWest = 0x1,
    WallNorth = 0x2,
    WallNorthEast = 0x4,
    WallEast = 0x8,
    WallSouthEast = 0x10,
    WallSouth = 0x20,
    WallSouthWest = 0x40,
    WallWest = 0x80,
    Loc = 0x100,
    WallNorthWestProjBlocker = 0x200,
    WallNorthProjBlocker = 0x400,
    WallNorthEastProjBlocker = 0x800,
    WallEastProjBlocker = 0x1000,
    WallSouthEastProjBlocker = 0x2000,
    WallSouthProjBlocker = 0x4000,
    WallSouthWestProjBlocker = 0x8000,
    WallWestProjBlocker = 0x10000,
    LocProjBlocker = 0x20000,
    FloorDecoration = 0x40000,

    /**
     * Custom flag dedicated to blocking NPCs.
     * It should be noted that this is a custom flag, and you do not need to use this.
     * The pathfinder takes the flag as a custom option, so you may use any other flag, this just defines
     * a reliable constant to use
     */
    Npc = 0x80000,

    /**
     * Custom flag dedicated to blocking players, projectiles as well as NPCs.
     * An example of a monster to set this flag is Brawler. Note that it is unclear if this flag
     * prevents NPCs, as there is a separate flag option for it.
     * This flag is similar to the one above, except it's strictly for NPCs.
     */
    Player = 0x100000,

    Floor = 0x200000,
    WallNorthWestRouteBlocker = 0x400000,
    WallNorthRouteBlocker = 0x800000,
    WallNorthEastRouteBlocker = 0x1000000,
    WallEastRouteBlocker = 0x2000000,
    WallSouthEastRouteBlocker = 0x4000000,
    WallSouthRouteBlocker = 0x8000000,
    WallSouthWestRouteBlocker = 0x10000000,
    WallWestRouteBlocker = 0x20000000,
    LocRouteBlocker = 0x40000000,

    /**
     * Roof flag, used to bind NPCs to not leave the buildings they spawn in. This is a custom flag.
     */
    Roof = 0x80000000,

    /* A shorthand combination of both the floor flags. */
    FloorBlocked = 0x240000,

    WalkBlocked = 0x240100,

    /* Mixed masks of the above flags */
    BlockWest = 0x240108,
    BlockEast = 0x240180,
    BlockSouth = 0x240102,
    BlockNorth = 0x240120,

    BlockSouthWest = 0x24010E,
    BlockSouthEast = 0x240183,
    BlockNorthWest = 0x240138,
    BlockNorthEast = 0x2401E0,
    BlockNorthAndSouthEast = 0x24013E,
    BlockNorthAndSouthWest = 0x2401E3,
    BlockNorthEastAndWest = 0x24018F,
    BlockSouthEastAndWest = 0x2401F8,

    /* Route blocker flags. These are used in ~550+ clients to generate paths through bankers and such. */
    BlockWestRouteBlocker = 0x42240000,
    BlockEastRouteBlocker = 0x60240000,
    BlockSouthRouteBlocker = 0x40A40000,
    BlockNorthRouteBlocker = 0x48240000,

    BlockSouthWestRouteBlocker = 0x43A40000,
    BlockSouthEastRouteBlocker = 0x60E40000,
    BlockNorthWestRouteBlocker = 0x4E240000,
    BlockNorthEastRouteBlocker = 0x78240000,

    BlockNorthAndSouthEastRouteBlocker = 0x4FA40000,
    BlockNorthAndSouthWestRouteBlocker = 0x78E40000,
    BlockNorthEastAndWestRouteBlocker = 0x63E40000,
    BlockSouthEastAndWestRouteBlocker = 0x7E240000,

    Null = 0x7FFFFFFF,
}
