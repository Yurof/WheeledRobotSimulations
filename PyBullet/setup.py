from setuptools import setup, find_packages

setup(
    name="iRobot_gym",
    version="0.9",
    packages=find_packages(),
    install_requires=['pybullet', 'scipy', 'numpy',
                      'gym', 'yamldataclassconfig', 'nptyping'],
)
