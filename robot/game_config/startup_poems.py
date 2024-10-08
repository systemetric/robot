from random import randint

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
        jokeNo = randint(0,len(POEM_ON_STARTUP.jokes))
        jokeToPrint = "I don't know what went wrong, but we messed up our joke loading ;-;"
        try:
            jokeToPrint = POEM_ON_STARTUP.jokes[jokeNo]
        except:
            jokeToPrint = POEM_ON_STARTUP.jokes[0]
        logger.info(jokeToPrint)