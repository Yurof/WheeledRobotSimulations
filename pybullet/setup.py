from setuptools import setup, find_packages

setup(
    name="racecar_gym",
    version="0.1",
    packages=find_packages(),
    install_requires=['pybullet', 'scipy', 'numpy', 'gym', 'yamldataclassconfig', 'nptyping'],
)