import enum

"""
This defines each team, and a corresponding Tx value for the team (x is the team index). Make sure that 
the value of the Tx matches its team, and is unique - this year I picked the latin names for the potato varieties
that the school will be competing as.
"""
class TEAM(enum.Enum): 
    RUSSET = "Solanum Tuberosum 'Ranger Russet'" # This matches T0, for example.
    SWEET = "Ipomoea batatas"
    MARIS_PIPER = "Solanum Tuberosum 'Maris Piper'"
    PURPLE = "Solanum Tuberosum 'Vitolette'"
    ARENA = "HOTTTTT!" 
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    T0 = "Solanum Tuberosum 'Ranger Russet'"
    T1 = "Ipomoea batatas"
    T2 = "Solanum Tuberosum 'Maris Piper'"
    T3 = "Solanum Tuberosum 'Vitolette'"

