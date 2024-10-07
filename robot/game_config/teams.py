import enum

"""
This defines each team, and a corresponding Tx value for the team (x is the team index). Make sure that 
the value of the Tx matches its team, and is unique - this year I picked the latin names for the potato varieties
that the school will be competing as.
"""
class TEAM(enum.Enum):
# TODO (2025): Make these names better    
    TEAM1 = "TEAM1"
    TEAM2 = "TEAM2"
    TEAM3 = "TEAM3"
    TEAM4 = "TEAM4"
    ARENA = "ARENA" 
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    T0 = "TEAM1"
    T1 = "TEAM2"
    T2 = "TEAM3"
    T3 = "TEAM4"

