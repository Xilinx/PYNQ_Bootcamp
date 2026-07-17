from setuptools import setup, find_packages

setup(
    name="genesis_server",
    version="0.1.0",
    author="PYNQ Bootcamp",
    description="Genesis simulation server for PYNQ remote robotics with video streaming",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "genesis-world",
        "numpy",
        "flask>=2.0.0",
        "opencv-python",
        "pillow",
    ],
    entry_points={
        "console_scripts": [
            "genesis-server=genesis_server.server:main",
        ],
    },
)
