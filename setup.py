from setuptools import setup, find_packages

setup(
    name="robot",
    version="2024.1",
    packages=find_packages(),

    install_requires=[
        "smbus2>=0.4.2",
        "fake-rpi>=0.7.1",
        "opencv-python-headless~=3.4.0",
        "scipy>=1.9.1",
        "numpy<2.0.0",
    ],

    author="Skyler Grey",
    author_email="skyler3665@gmail.com",
)
