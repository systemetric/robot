import enum
import typing

from .teams import TEAM


class MARKER_TYPE(enum.Enum):
    POTATO = enum.auto()
    ARENA = enum.auto()


# class MARKER_OWNER(enum.Enum):
#     ME = enum.auto()
#     ARENA = enum.auto()
#     ANOTHER_TEAM = enum.auto()


class BASE_MARKER:
    team_marker_colors: dict = {
        TEAM.RUSSET: (64, 255, 0), # RED
        TEAM.MARIS_PIPER: (128, 0, 255), # GREEN
        TEAM.PURPLE: (255, 32, 32), # PURPLE
        TEAM.SWEET: (255, 255, 32), # YELLOW
    }

    def __init__(
        self,
        id: int,
        type: MARKER_TYPE,
        # owner: MARKER_OWNER,
    ) -> None:
        self.id = id

        self.type = type
        # self.owner = owner

        self.owning_team: typing.Union[TEAM, None] = None

        # self.wool_type: WOOL_TYPE | None = None

        # Sizes are in meters
        self.size = 0.2 if self.type == MARKER_TYPE.ARENA else 0.08

    def __repr__(self) -> str:
        return f"<Marker type={self.type} owning_team={self.owning_team} />"

    @property
    def bounding_box_color(self) -> tuple:
        if self.type == MARKER_TYPE.ARENA:
            return tuple(reversed((125, 249, 225))) # turquoise
        elif self.owning_team==TEAM.ARENA:
            return tuple(reversed(255,255,255)) # white
        else:
            return tuple(reversed(self.team_marker_colors[self.owning_team])) # team colour

class ARENA_MARKER(BASE_MARKER):
    def __init__(self, id: int) -> None:
        super().__init__(id, MARKER_TYPE.ARENA)

    def __repr__(self) -> str:
        return f"<Marker(ARENA)/>"


class POTATO_MARKER(BASE_MARKER):
    def __init__(
        self, id: int, owner: TEAM, #my_team: typing.Union[TEAM, None], wool_type: WOOL_TYPE
    ) -> None:
        super().__init__(
            id,
            MARKER_TYPE.POTATO,
        )
        self.owning_team = owner

        # self.wool_type = wool_type

    def __repr__(self) -> str:
        return f"<Marker(POTATO) owning_team={self.owning_team} />"


class MARKER(BASE_MARKER):
    @staticmethod
    def by_id(id: int, team: typing.Union[TEAM, None] = None) -> BASE_MARKER:
        """
        Get a marker object from an id

        Marker IDs are between 0 and 99(? potentially higher)
        Low marker IDs (0-39) are potatoes. The first 4 potato markers are Jacket 
        Potatoes and belong to the team with the corresponding ID.
        The rest of the low markers are unowned - their owning_team property is None
        and their owner is ARENA.

        Higher marker IDs (40+) will be part of the arena. These markers will be
        spaced as in 2021-2022's competition (6 markers on each side of the Arena,
        the first 50cm away from the wall and each subsequent marker 1m away from
        there). In practice these markers start at 100.

        If no team is provided, all team-owned markers will be assumed to belong to
        ANOTHER_TEAM
        """

        if id >= 40:
            return ARENA_MARKER(id)
        
        wrappingId = id % 20
        if wrappingId<4:
            owning_team = TEAM[f"T{wrappingId}"]
        else:
            owning_team = TEAM["ARENA"]

        return POTATO_MARKER(id, owning_team)