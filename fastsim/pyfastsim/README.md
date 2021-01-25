pyFastsim
=========

Python bindings for [libfastsim](https://github.com/jbmouret/libfastsim).


Requirements
------------
- **A c++14 capable compiler is needed !**
- the pybind11 python module
- the pybind11 development package (libpybind11-dev on Debian/Ubuntu)
- libfastsim from github (https://github.com/jbmouret/libfastsim), patched with the provided patch. The patch does does the following things:
  * Adds -fPIC flag to compilation
  * Migrates the code from boost::shared\_ptr to std::shared\_ptr (which is much easier to use with pybind11)
  * Normalizes which objects are accessed directly or through std::shared\_ptr
  * Renames the const version of Robot::use\_camera to Robot::camera_enabled
  * Adds a few missing constructors and methods necessary to fully expose objects states and therefore make objects picklables
  * (Removes debug flags and make the test more verbose; that is optional)
  

Installation
------------
- Checkout, patch and install [libfastsim](https://github.com/jbmouret/libfastsim).
- Run `pip3 install .` from the directory



Usage
-----
You can run the example with `python3 example.py worlds/example.xml` from the example repository

It is a port of the C++ test_fastsim and should give exactly the same results.
