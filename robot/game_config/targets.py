import enum

"""
Defines each target type, for example sheep, lair or gem.
"""
class TARGET_TYPE(enum.Enum): 
    
    SUPPLY_L = "SUPPLY_L" # This matches T0, for example.
    SUPPLY_H = "SUPPLY_H"
    
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    NONE = "NONE"  ## This is only used when no other owning team is available for undefined IDs, this should never actually happen in-game.

    T0 = "SUPPLY_L"
    T1 = "SUPPLY_H"

