import enum

"""
This defines each team, and a corresponding Tx value for the team (x is the team index). Make sure that 
the value of the Tx matches its team, and is unique.
"""
class TEAM(enum.Enum): 
    RUBY = "Ruby" # This matches T0, for example.
    DIAMOND = "Diamond"
    JADE = "Jade"
    TOPAZ = "Topaz"
    ARENA = "Arena" 
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    T0 = "Ruby"
    T1 = "Diamond"
    T2 = "Jade"
    T3 = "Topaz"

