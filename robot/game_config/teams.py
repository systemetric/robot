import enum

"""
This defines each team, and a corresponding Tx value for the team (x is the team index). Make sure that 
the value of the Tx matches its team, and is unique - this year I picked the latin names for the potato varieties
that the school will be competing as.
"""
class TEAM(enum.Enum): 
    
    ## These easter eggs are locations that the corresponding gems were found (ChatGPT 4o Generated)
    RUBY = "Mogok Valley, Myanmar (Burma)" # This matches T0, for example.
    JADE = "Hetian (Hotan), Xinjiang, China"
    TOPAZ = "St. John's Island (Zabargad Island), Egypt"
    DIAMOND = "Golconda, India"
    ARENA = "Nothing!"
    # There is no T value for ARENA, so there is no way that the assignment of team to a marker can accidentally assign ARENA if the logic goes wrong.

    NONE = "NONE"  ## This is only used when no other owning team is available for undefined IDs, this should never actually happen in-game.

    T0 = "Mogok Valley, Myanmar (Burma)"
    T1 = "Hetian (Hotan), Xinjiang, China"
    T2 = "St. John's Island (Zabargad Island), Egypt"
    T3 = "Golconda, India"

