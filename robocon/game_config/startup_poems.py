from random import randint

class POEM_ON_STARTUP:
    ## OUT OF DATE
    jokes = [
        "Why don't dragons tell secrets? \
            Because they always breathe fire!",
        "Why did the ruby go to school? \
            To become a little more polished!",
        "Why do sheep make terrible detectives? \
            Because they always follow the herd!",
        "What's a dragon's favourite type of exercise? \
            Flame-ups!",            
        "What did the diamond say to the ruby at the party? \
            You're looking radiant tonight!",            
        "Why was the sheep so quiet? \
            Because it was feeling a little wool-gathered!",  
    ]

    @staticmethod
    def on_startup(logger, random):
        """
        This is called on startup. Put something funny and relevant to this 
        years competition using the logger. Also random is currently passed 
        as an argument because I don't have the energy to try importing it,
        I just spent quite a while struggling with the new brains.
        """
        jokeNo = randint(0,len(POEM_ON_STARTUP.jokes))
        jokeToPrint = "I don't know what went wrong, but we messed up our joke loading ;-;"
        try:
            jokeToPrint = POEM_ON_STARTUP.jokes[jokeNo]
        except:
            jokeToPrint = POEM_ON_STARTUP.jokes[0]
        #logger.info(jokeToPrint)
