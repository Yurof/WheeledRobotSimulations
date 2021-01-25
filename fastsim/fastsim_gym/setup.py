from setuptools import setup, find_packages

setup(name='gym_fastsim',
      version='0.0.5',
      install_requires=['gym>=0.11.0','pyfastsim'],
      packages=find_packages(include=['gym_fastsim', 'gym_fastsim.*']),
      package_data={'gym_fastsim':['assets/*']},
      author='Alex Coninx',
      author_email='coninx@isir.upmc.fr'
)
