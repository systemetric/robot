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

Hello
I've updated this for 2025 and renamed the movable boxes to BOXs. This is just an internal name and
mostly shouldn't be visible to the users. In 2025 they are dealing with Sheep, Gems and Lair markers.
- OTL (2024-2025)
"""

class MARKER_TYPE(enum.Enum): # Keep something like this to determine if a marker is a wall or not.
    BOX = enum.auto()    # This is a movable box. In 2025 this is a Sheep or a Gem
    ARENA = enum.auto()     # These are the wall markers
    LAIR = enum.auto()      # In 2025 each team has a lair with a small marker on the wall



class BASE_MARKER: # Base marker class that BOX_MARKER and ARENA_MARKER derive from.
    team_marker_colors: dict = { # Colour definitions for each team defined in TEAMS
        TEAM.RUBY: (255, 64, 0), # RED
        TEAM.DIAMOND: (255, 255, 32), # YELLOW
        TEAM.JADE: (50,255,0), # GREEN
        TEAM.TOPAZ: (0, 100, 255), # BLUE
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
            return tuple((125, 249, 225)) # Turquoise
        elif self.owning_team==TEAM.ARENA: # If it is a Sheep (game object owned by ARENA)
            return tuple((55,255,255)) # White
        elif self.owning_team: # If it is a Gem (game object owned by a team.)
            return tuple(self.team_marker_colors[self.owning_team]) # Picks the team colour from above
        else: # No owning team?
            return tuple((255,125,125)) # Pinky


class ARENA_MARKER(BASE_MARKER): # Not much going on here. This represents a wall.
    def __init__(self, id: int) -> None:
        super().__init__(id, MARKER_TYPE.ARENA)

    def __repr__(self) -> str:
        return f"<Marker(ARENA)/>"


class BOX_MARKER(BASE_MARKER): # This is a game object rather than a wall. Add properties you want to keep track of
    def __init__(
        self, id: int, owner: TEAM
    ) -> None:
        super().__init__(id, MARKER_TYPE.BOX)
        self.owning_team = owner

    def __repr__(self) -> str:
        return f"<Marker(BOX) owning_team={self.owning_team} />"

class LAIR_MARKER(BASE_MARKER): # This is marks a teams lair so is a wall but also has an owning team.
    def __init__(
        self, id: int, owner: TEAM
    ) -> None:
        super().__init__(id, MARKER_TYPE.LAIR)
        self.owning_team = owner

    def __repr__(self) -> str:
        return f"<Marker(LAIR) owning_team={self.owning_team} />"

class MARKER(BASE_MARKER): # This is literally just how the code gets the different marker types.
    @staticmethod
    def by_id(id: int, team: typing.Union[TEAM, None] = None) -> BASE_MARKER: # team is currently unused, but it is referenced throughout the code. It is the team of the robot I believe (check this)
        """
        Get a marker object from an id

        Marker IDs are greater than 0, and usually won't go higher than 200. They are 
        read as April tags, so use an online 6x6 April tag generator to test values, 
        such as:
        https://chaitanyantr.github.io/apriltag (remember to set to tag36h11 for this one)

        In 2025 low marker IDs (0-49) are Sheep and Gems.

        Markers 20-23 and 40-43 are Gems, with a pair of markers for each team.

        The rest of the low markers are unowned sheep - their owning_team property is ARENA.

        It is probably recommendable that you have duplicate marker IDs, so that any damaged
        marker can be replaced by one with equivalent game meaning - in 2024 we made ID 0 and 
        ID 20 equivalent.

        Higher marker IDs (40+) will be part of the arena. These markers will be
        spaced as in 2021-2022's competition (6 markers on each side of the Arena,
        the first 50cm away from the wall and each subsequent marker 1m away from
        there). In practice these markers start at 100.

        In 2025 there are also Lair markers, one of each of the team's lairs. These are numbered 50-53.
        """

        ARENA_WALL_LOWER_ID = 60
        if id >= ARENA_WALL_LOWER_ID:
            return ARENA_MARKER(id)
        
        elif id >=50:
            owning_team = TEAM[f"T{id%50}"] # Set to the corresponding TEAM enum.
            return LAIR_MARKER(id, owning_team)


        wrappingId = id % 20 # Make sure that the ID range wraps after 20 values.
        if wrappingId<4 and id >= 20: # If it is a Gem (has a team)
            owning_team = TEAM[f"T{wrappingId}"] # Set to the corresponding TEAM enum.
        else: # If it is a Sheep (Has no team)
            owning_team = TEAM["ARENA"]

        return BOX_MARKER(id, owning_team)


