from random import randint

class POEM_ON_STARTUP:
    jokes = [
        "In some whimsical traditions, it is joked that rain is caused by clouds of weeping dragons whose favorite gems have gone missing and those that find the gems are rewarded with clear skies.",
        "Dragons are rumored to enjoy a special 'stone soup' made with sapphires, emeralds, and other precious stones, which they claim enhances a flame's color when they breathe fire.",
        "The myth about some flying dragons is that they leave colorful trails in the sky similar to a rainbow caused by light refracting through gemstones embedded in their scales.",
        "In certain cultures, small guardian creatures called gemlings are said to live in gemstones, tasked with keeping them safe from thieves, even from dragons themselves.",
        "Some dragon myths humorously claim that the sparkle of gems and gold in a dragon's lair is what inspired disco balls—dragons were the first disco dancers, spinning under their self-made glimmering lights!",
        "Some folklore suggests that dragons possess an incredible talent for jewelry crafting and often make intricate necklaces and crowns from the gems they hoard."
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