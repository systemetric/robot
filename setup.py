from setuptools import setup

setup(
    name="robot",
    version="2024.1",
    packages=["robot"],

    install_requires=[
        "smbus2>=0.4.2",
        "fake-rpi>=0.7.1",
        "opencv3>=3.4.18",
        "scipy>=1.9.1",
        "wiringpi>=2.60.1",
    ],

    author="Skyler Grey",
    author_email="skyler3665@gmail.com",
)
