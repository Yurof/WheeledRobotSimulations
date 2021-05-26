#!/bin/sh

# Alex Coninx
# ISIR - Sorbonne Universite / CNRS
# 7/10/2019

# Dependecies : boost, sdl, C++11 compatible compiler, python3, pip3, python2.7,
# git
set -o errexit
set -o nounset
set -o pipefail



BASE_DIR="${PWD}"

# Create dir
mkdir -p "${BASE_DIR}/src"

echo
echo "====================================="
echo "===== (1/7) Installing gym ====="
echo "====================================="
echo
pip3 install gym

echo
echo "====================================="
echo "===== (2/7) Installing deap ====="
echo "====================================="
echo
pip3 install deap

# 1) Install pybind11
echo
echo "====================================="
echo "===== (3/7) Installing pybind11 ====="
echo "====================================="
echo

cd "${BASE_DIR}/src"

if [ ! -d "pybind11" ] ; then
    git clone https://github.com/pybind/pybind11.git pybind11
    cd "pybind11"
else
    cd "pybind11"
    git pull https://github.com/pybind/pybind11.git
fi
# Install the pybind11 python module
pip3 install .
# Where we can find pybind11 (especially its includes)
PYBIND11_DIR="${BASE_DIR}/src/pybind11"
 
# 2) Install and patch fastsim
echo
echo "===================================================="
echo "===== (4/7) Patching and installing libfastsim ====="
echo "===================================================="
echo
cd "${BASE_DIR}/src"

if [ ! -d "pyfastsim" ] ; then
    git clone https://github.com/alexendy/pyfastsim.git pyfastsim
else
    cd "pyfastsim"
    git pull https://github.com/alexendy/pyfastsim.git
fi
cd "${BASE_DIR}/src"


if [ ! -d "libfastsim" ] ; then
    git clone https://github.com/jbmouret/libfastsim.git libfastsim
    cd libfastsim
    patch -p1 < ../pyfastsim/fastsim-boost2std-fixdisplay.patch
else
    cd "libfastsim"
    git pull https://github.com/jbmouret/libfastsim.git
fi

# Build and install
python2.7 ./waf configure --prefix=./install
python2.7 ./waf build
python2.7 ./waf install
# Where we installed fastsim
FASTSIM_DIR="${BASE_DIR}/src/libfastsim/install"

# 3) Install pyfastsim
echo
echo "======================================"
echo "===== (5/7) Installing pyfastsim ====="
echo "======================================"
echo
cd "${BASE_DIR}/src/pyfastsim"
CPPFLAGS="-I\"${PYBIND11_DIR}/include\" -I\"${FASTSIM_DIR}/include\"" LDFLAGS="-L\"${FASTSIM_DIR}/lib\"" pip3 install .

# 1) Install fastsim_gym
echo
echo "====================================="
echo "===== (6/7) Installing fastsim_gym ====="
echo "====================================="
echo
cd "${BASE_DIR}/fastsim"

pip3 install .

cd "${BASE_DIR}/src"
echo
echo "======================================"
echo "===== (7/7) Installing pybullet-gym ====="
echo "======================================"
echo
if [ ! -d "pybullet-gym" ] ; then
    git clone https://github.com/benelot/pybullet-gym.git pybullet-gym
    cd pybullet-gym
else
    cd "pybullet-gym"
    git pull https://github.com/benelot/pybullet-gym.git
fi

pip install -e .


