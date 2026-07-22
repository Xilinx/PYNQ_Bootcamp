from setuptools import setup, find_packages

setup(
    name="pynqsim",
    version="0.1.0",
    author="PYNQ Bootcamp",
    description="Client for Genesis remote simulation server",
    packages=find_packages(),
    python_requires=">=3.6",
    install_requires=[
        "requests",
    ],
)
