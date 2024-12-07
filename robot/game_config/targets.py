import enum

"""
Defines each target type, for example sheep, lair or gem.
"""
class TARGET_TYPE(enum.Enum): 
    
    ## These easter eggs are locations that the corresponding gems were found (ChatGPT 4o Generated)
    SHEEP = "Baa" # This matches T0, for example.
    GEM = "Shiny"
    LAIR = "Dangerous"
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    NONE = "NONE"  ## This is only used when no other owning team is available for undefined IDs, this should never actually happen in-game.

    T0 = "Baa"
    T1 = "Shiny"
    T2 = "Dangerous"

