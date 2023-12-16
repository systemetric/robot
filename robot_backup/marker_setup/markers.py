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

[Put your future messages here]
"""

class POEM_ON_STARTUP:
    jokes = [
        "Why did the potato cross the road? \
            He saw a fork up ahead.",
        "What do you say to a baked potato that's angry? \
            Anything you like, just butter it up.",
        "Funny Potato Joke",
        "Farm Facts: There are around 5000 different varieties of potato",
        "Farm Facts: Potatoes were first domesticated in Peru around 4500 years ago around Lake Titicaca",
        "Farm Facts: The word potato originates from the Taino \"batata\", meaning sweet potato.",
        "Farm Facts: China is the leading producer of potatoes, with 94.3 million tonnes produced in 2021",
        "Farm Facts: The maximum theoretical voltage "
    ]

    @staticmethod
    def on_startup(logger, random):
        """
        This is called on startup. Put something funny and relevant to this 
        years competition using the logger. Also random is currently passed 
        as an argument because I don't have the energy to try importing it,
        I just spent quite a while struggling with the new brains.
        """
        jokeNo = random.randint(0,len(POEM_ON_STARTUP.jokes))
        jokeToPrint = "I don't know what went wrong, but we messed up our joke loading ;-;"
        try:
            jokeToPrint = POEM_ON_STARTUP.jokes[jokeNo]
        except:
            jokeToPrint = POEM_ON_STARTUP.jokes[0]
        logger.info(jokeToPrint)

class MARKER_TYPE(enum.Enum): # Keep something like this to determine if a marker is a wall or not.
    POTATO = enum.auto()
    ARENA = enum.auto()


class BASE_MARKER: # Base marker class that POTATO_MARKER and ARENA_MARKER derive from.
    team_marker_colors: dict = { # Colour definitions for each team defined in TEAMS
        TEAM.RUSSET: (255, 64, 0), # RED
        TEAM.SWEET: (255, 255, 32), # YELLOW
        TEAM.MARIS_PIPER: (50,255,0), # GREEN
        TEAM.PURPLE: (255, 32, 255), # PURPLE
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
            return tuple(reversed((125, 249, 225))) # Turquoise
        elif self.owning_team==TEAM.ARENA: # If it is a Hot Potato (game object owned by ARENA)
            return tuple(reversed((255,255,255))) # White
        else: # If it is a Jacket Potato (game object owned by a team.)
            return tuple(reversed(self.team_marker_colors[self.owning_team])) # Picks the team colour from above

class ARENA_MARKER(BASE_MARKER): # Not much going on here. This represents a wall.
    def __init__(self, id: int) -> None:
        super().__init__(id, MARKER_TYPE.ARENA)

    def __repr__(self) -> str:
        return f"<Marker(ARENA)/>"


class POTATO_MARKER(BASE_MARKER): # This is a game object rather than a wall. Add properties you want to keep track of
    def __init__(
        self, id: int, owner: TEAM
    ) -> None:
        super().__init__(id, MARKER_TYPE.POTATO)
        self.owning_team = owner

    def __repr__(self) -> str:
        return f"<Marker(POTATO) owning_team={self.owning_team} />"

class MARKER(BASE_MARKER): # This is literally just how the code gets the different marker types.
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
        """

        ARENA_WALL_LOWER_ID = 40
        if id >= ARENA_WALL_LOWER_ID:
            return ARENA_MARKER(id)
        
        wrappingId = id % 20 # Make sure that the ID range wraps after 20 values.
        if wrappingId<4: # If it is a Jacket Potato (has a team)
            owning_team = TEAM[f"T{wrappingId}"] # Set to the corresponding TEAM enum.
        else: # If it is a Hot Potato (Has no team)
            owning_team = TEAM["ARENA"]

        return POTATO_MARKER(id, owning_team)
