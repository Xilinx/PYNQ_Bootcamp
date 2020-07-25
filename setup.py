from setuptools import find_packages, setup

setup(
    name="pmod_grove_aht20",
    version='1.0',
    install_requires=[
        'pynq>=2.5',
    ],
    url='https://github.com/LewisMcL/PYNQ_Bootcamp',
    license='BSD 3-Clause License',
    author="Lewis Davin McLaughlin",
    author_email="lewis.mclaughlin@strath.ac.uk",
    packages=find_packages(),
    package_data={
    },
    description="Python driver for Grove AHT20 peripheral.")
