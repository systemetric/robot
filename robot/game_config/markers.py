import enum
import typing

from .teams import TEAM

"""
Hiiii!
This file contains the definitions for the markers and the data we assign to them. 
I have just set them to 2024's competition values, with MARKER_TYPE deciding whether a marker is part of 
the ARENA walls, or a game object, which is this year (2023-2024) called a POTATO. So make the changes you 
need! But make sure to change every reference to it in the code, not just the ones in this file.
So go nuts! Good luck with your Robocon, and feel free to add your own messages for the future below!
Byee!
 - Holly (2023-2024)


Don't know what to say. - Nathan Gill (2024 - 2025)
No semi-colons please - Scott Wilson (2024 - 2025)

[Put your future messages here]
"""

class MARKER_TYPE(enum.Enum): # Keep something like this to determine if a marker is a wall or not.
    TARGET = enum.auto()
    ARENA = enum.auto()


class BASE_MARKER: # Base marker class that TARGET_MARKER and ARENA_MARKER derive from.
    team_marker_colors: dict = { # Colour definitions for each team defined in TEAMS
        TEAM.RUBY: (0, 0, 255), # RED
        TEAM.JADE: (0, 255, 0), # GREEN
        TEAM.TOPAZ: (0, 255, 255), # YELLOW
        TEAM.DIAMOND: (255, 0, 0), # BLUE
    }

    def __init__(
        self,
        id: int,
        type: MARKER_TYPE,
    ) -> None:
        self.id = id
        self.type = type
        self.owning_team: typing.Union[TEAM, None] = None

        # Sizes are in meters
        self.size = 0.2 if self.type == MARKER_TYPE.ARENA else 0.08

    def __repr__(self) -> str:
        return f"<Marker type={self.type} owning_team={self.owning_team} />"

    @property
    def bounding_box_color(self) -> tuple:
        if self.type == MARKER_TYPE.ARENA: # If it is a wall
            return tuple(reversed((225, 249, 125))) # Turquoise
        elif self.owning_team==TEAM.ARENA: # If it is a Sheep (game object owned by ARENA)
            return tuple(reversed((255,255,255))) # White
        else: # If it is a Jewel (game object owned by a team.)
            return tuple(reversed(self.team_marker_colors[self.owning_team])) # Picks the team colour from above

class ARENA_MARKER(BASE_MARKER): # Not much going on here. This represents a wall.
    def __init__(self, id: int) -> None:
        super().__init__(id, MARKER_TYPE.ARENA)

    def __repr__(self) -> str:
        return f"<Marker(ARENA)/>"


class TARGET_MARKER(BASE_MARKER): # This is a game object rather than a wall. Add properties you want to keep track of
    def __init__(
        self, id: int, owner: TEAM
    ) -> None:
        super().__init__(id, MARKER_TYPE.TARGET)
        self.owning_team = owner

    def __repr__(self) -> str:
        return f"<Marker(TARGET) owning_team={self.owning_team} />"

class MARKER(BASE_MARKER): # This is literally just how the code gets the different marker types.
    @staticmethod
    def get_size(id: int):
        tg = MARKER.by_id(id)
        if tg.owning_team == TEAM["ARENA"]:
            return 0.2
        return 0.08
    
    @staticmethod
    def by_id(id: int, team: typing.Union[TEAM, None] = None) -> BASE_MARKER: # team is currently unused, but it is referenced throughout the code. It is the team of the robot I believe (check this)
        """
        Get a marker object from an id

        Marker IDs are greater than 0, and usually won't go higher than 200. They are 
        read as April tags, so use an online 6x6 April tag generator to test values, 
        such as:
        https://chaitanyantr.github.io/apriltag (remember to set to tag36h11 for this one)

        In 2024 low marker IDs (0-39) are potatoes. The first 4 potato markers are Jacket 
        Potatoes and belong to the team with the corresponding ID.
        The rest of the low markers are unowned - their owning_team property is None
        and their owner is ARENA.

        It is probably recommendable that you have duplicate marker IDs, so that any damaged
        marker can be replaced by one with equivalent game meaning - in 2024 we made ID 0 and 
        ID 20 equivalent.

        Higher marker IDs (40+) will be part of the arena. These markers will be
        spaced as in 2021-2022's competition (6 markers on each side of the Arena,
        the first 50cm away from the wall and each subsequent marker 1m away from
        there). In practice these markers start at 100.
        
        
        As of 2025: Sheep have IDs 0-23 inclusive; Jewels have IDs 24-31 inclusive; Lair IDs are 50-53 inclusive
        
        """

        if id > 99:
            return ARENA_MARKER(id)

        if id < 24:
            owning_team = TEAM["ARENA"]
        elif id == 30 or id == 31 or id == 53:
            owning_team = TEAM["T3"]
        elif id == 28 or id == 29 or id == 52:
            owning_team = TEAM["T2"]
        elif id == 26 or id == 27 or id == 51:
            owning_team = TEAM["T1"]
        elif id == 24 or id == 25 or id == 50:
            owning_team = TEAM["T0"]
        else:
            print(f"Marker ID {id} is not defined.")
            owning_team = TEAM["NONE"]

        return TARGET_MARKER(id, owning_team)