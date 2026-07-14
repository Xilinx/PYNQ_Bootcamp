from setuptools import setup, find_packages

setup(
    name="genesis_server",
    version="0.1.0",
    author="PYNQ Bootcamp",
    description="Genesis simulation server for PYNQ remote robotics",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "genesis-world",
        "numpy",
    ],
    entry_points={
        "console_scripts": [
            "genesis-server=genesis_server.server:main",
        ],
    },
)
