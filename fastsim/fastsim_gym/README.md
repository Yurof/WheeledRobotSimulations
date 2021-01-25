# Gym environment for pyfastsim

The default environment is:
- Lehman & Stanley's hard maze (dimension 600x600)
- Robot with 3 lasers at -pi/4, 0 and -pi/4, range 100, and two bumpers




## Requirements
- pyfastsim
- gym

## Usage

See example_gym_fastsim.py

## Miscellaneous

The XML file describing the environment and robot must contain the path to the map file. The path can be absolute or relative to the location of the XML file.

The maps must be
* Binary PBM format
* Square
* With size divisible by 8
