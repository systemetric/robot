from random import randint

class POEM_ON_STARTUP:
    jokes = [
        "In some whimsical traditions, it is joked that rain is caused by clouds of weeping dragons whose favorite gems have gone missing and those that find the gems are rewarded with clear skies.",
        "Dragons are rumored to enjoy a special 'stone soup' made with sapphires, emeralds, and other precious stones, which they claim enhances a flame's color when they breathe fire.",
        "The myth about some flying dragons is that they leave colorful trails in the sky similar to a rainbow caused by light refracting through gemstones embedded in their scales.",
        "In certain cultures, small guardian creatures called gemlings are said to live in gemstones, tasked with keeping them safe from thieves, even from dragons themselves.",
        "Some dragon myths humorously claim that the sparkle of gems and gold in a dragon's lair is what inspired disco balls—dragons were the first disco dancers, spinning under their self-made glimmering lights!",
        "Some folklore suggests that dragons possess an incredible talent for jewelry crafting and often make intricate necklaces and crowns from the gems they hoard.",
        "In some whimsical traditions, it is joked that rain is caused by clouds of weeping dragons whose favorite gems have gone missing and those that find the gems are rewarded with clear skies.",
        "In an alternate legend, gems are believed to be shapeshifted dragon scales, which explains why dragons guard them so fiercely—they're just keeping track of their lost parts!",
        "Anecdotes tell of dragons holding friendly gem-eating contests, where they pretend to munch on gems, vying for the title of 'Most Voracious Hoover of Hoards.'",
        "A charming story suggests that when dragons dance under the moonlight, the gems in their hoards sparkle so brightly they create patterns believed to inspire human ornamental designs.",
        "Dragons are known to love baths. They fill pools with gems, believing that the reflected light enhances their scales' shine, akin to a dragon spa day.",
        "It's said that dragons and sheep sometimes engage in friendly games of chase, with the sheep trying to outrun the lumbering dragons in a race for speed and agility.",
        "According to a humorous myth, dragons secretly run underground 'baa-tiques' where sheep are treated to the finest spa services, from wool trims to luxurious mud baths.",
        "Ever-practical dragons are said to have used sheep as natural alarm systems, since the sheep would baa loudly in groups if intruders (or smaller, sneaky dragons) approached the treasure hoards.",
        "A fun legend suggests that young dragons make fluffy pillows out of sheep wool, claiming that nothing beats a wool-stuffed pillow when it comes to snoozing atop a pile of treasure.",
        "Elder dragons reportedly love regaling young drakes with tales of clever sheep that once outsmarted and outran them, serving as lessons in patience and humility."
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