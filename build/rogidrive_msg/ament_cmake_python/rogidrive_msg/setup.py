from setuptools import find_packages
from setuptools import setup

setup(
    name='rogidrive_msg',
    version='0.0.0',
    packages=find_packages(
        include=('rogidrive_msg', 'rogidrive_msg.*')),
)