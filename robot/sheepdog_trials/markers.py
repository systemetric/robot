import enum
import typing

from .teams import TEAM


class MARKER_TYPE(enum.Enum):
    SHEEP = enum.auto()
    ARENA = enum.auto()


class MARKER_OWNER(enum.Enum):
    ME = enum.auto()
    ARENA = enum.auto()
    ANOTHER_TEAM = enum.auto()


class WOOL_TYPE(enum.Enum):
    GOLDEN_FLEECE = enum.auto()
    STEEL_WOOL = enum.auto()


class BASE_MARKER:
    team_marker_colors: dict = {
        TEAM.LEON: (64, 255, 0),
        TEAM.PRIS: (128, 0, 255),
        TEAM.ROY: (255, 32, 32),
        TEAM.ZHORA: (255, 255, 32),
    }

    wool_type_colors: dict = {
        WOOL_TYPE.GOLDEN_FLEECE: (218, 165, 32),
        WOOL_TYPE.STEEL_WOOL: (113, 121, 126),
    }

    def __init__(
        self,
        id: int,
        type: MARKER_TYPE,
        owner: MARKER_OWNER,
    ) -> None:
        self.id = id

        self.type = type
        self.owner = owner

        self.owning_team: typing.Union[TEAM, None] = None

        self.wool_type: WOOL_TYPE | None = None

        # Sizes are in meters
        self.size = 0.2 if self.type == MARKER_TYPE.ARENA else 0.08

    def __repr__(self) -> str:
        return f"<Marker type={self.type} wool_type={self.wool_type} owner={self.owner} owning_team={self.owning_team} />"

    @property
    def bounding_box_color(self) -> tuple:
        if self.type == MARKER_TYPE.ARENA:
            return 125, 249, 225

        if self.owner == MARKER_OWNER.ANOTHER_TEAM:
            assert self.owning_team != None
            # We cannot have another team owning the marker but no owning team
            return self.team_marker_colors[self.owning_team]

        assert self.wool_type != None
        # If MARKER_TYPE == SHEEP, we must have a wool_type
        return self.wool_type_colors[self.wool_type]

class ARENA_MARKER(BASE_MARKER):
    def __init__(self, id: int) -> None:
        super().__init__(id, MARKER_TYPE.ARENA, MARKER_OWNER.ARENA)

    def __repr__(self) -> str:
        return f"<Marker(ARENA)/>"


class SHEEP_MARKER(BASE_MARKER):
    def __init__(
        self, id: int, owner: TEAM, my_team: typing.Union[TEAM, None], wool_type: WOOL_TYPE
    ) -> None:
        super().__init__(
            id,
            MARKER_TYPE.SHEEP,
            MARKER_OWNER.ANOTHER_TEAM if owner != my_team else MARKER_OWNER.ME,
        )
        self.owning_team = owner

        self.wool_type = wool_type

    def __repr__(self) -> str:
        return f"<Marker(SHEEP) wool_type={self.wool_type} owning_team={self.owning_team} />"


class MARKER(BASE_MARKER):
    @staticmethod
    def by_id(id: int, team: typing.Union[TEAM, None] = None) -> BASE_MARKER:
        """
        Get a marker object from an id

        Marker IDs are between 0 and 99
        Low marker IDs (0-39) belong to teams. X0-X9 of each range of 10 may be
        assigned to sheep.

        For the markers which belong to sheep, low marker IDs (X0-X5) will be
        sheep with STEEL_WOOL. High marker IDs (X6-X9) will be sheep with
        GOLDEN_FLEECE.

        Higher marker IDs (40+) will be part of the arena. These markers will be
        spaced as in 2021-2022's competition (6 markers on each side of the Arena,
        the first 50cm away from the wall and each subsequent marker 1m away from
        there). In practice these markers start at 100.

        If no team is provided, all team-owned markers will be assumed to belong to
        ANOTHER_TEAM
        """

        if id >= 40:
            return ARENA_MARKER(id)

        owning_team = TEAM[f"T{id // 10}"]

        wool_type = WOOL_TYPE.GOLDEN_FLEECE if id % 10 > 5 else WOOL_TYPE.STEEL_WOOL

        return SHEEP_MARKER(id, owning_team, team, wool_type)
