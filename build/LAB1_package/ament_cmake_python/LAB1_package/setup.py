from setuptools import find_packages
from setuptools import setup

setup(
    name='LAB1_package',
    version='0.0.0',
    packages=find_packages(
        include=('LAB1_package', 'LAB1_package.*')),
)
