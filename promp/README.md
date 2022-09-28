# ProMP C++
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)

This is a library in C++ that implements Probabilistic Motion Primitives algorithms. References :

- Paraschos A, Daniel C, Peters J, Neumann G. Probabilistic movement primitives. Advances in neural information processing systems. 2013. [[pdf]](https://www.ias.informatik.tu-darmstadt.de/uploads/Publications/Paraschos_NIPS_2013.pdf)
- Paraschos A, Daniel C, Peters J, Neumann G. Using probabilistic movement primitives in robotics. Autonomous Robots. 2018 Mar;42(3):529-51. [[pdf]](https://www.ias.informatik.tu-darmstadt.de/uploads/Team/AlexandrosParaschos/promps_auro.pdf).


Initially, it was inspired in the following python implementations:
* [promps_python](https://github.com/mjm522/promps_python)
* [promplib](https://github.com/baxter-flowers/promplib)

## Requirements

* C++ 11
* CMake 3.11
* Eigen 3.2.92
* Doxygen 1.8.11 (optional)
* Python (optional)

(Other versions of CMake, Eigen, Doxygen will probably work, but those are the tested ones so far)


## Installation

```
git clone --recursive https://gitlab.inria.fr/H2020-AnDy/promp.git
cd promp
mkdir build
cd build
cmake ..
```
If you wish to compile the python library, set BUILD_PYTHON_MODULE to ON before compiling the library.

If you wish to build API documentation, set BUILD_DOCUMENTATIONS to ON before compiling the library (this requires doxygen and [doxybook2](https://github.com/matusnovak/doxybook2)).

Then, proceed to make and install:

```
make
sudo make install
```

If while executing your compiled targets you get an error like the following:
> cannot open shared object file: No such file or directory

Please try running ldconfig before running again:
```
sudo ldconfig
```

At the moment, the library has been tested on Ubuntu 16.04 and 18.04.

## Usage
You can refer to the source code of some examples in the `examples` folder to understand the usage of the library. Their binaries are automatically generated after the installation.

The api is described in [doc/api.md](doc/api.md)

In the folder `etc/demos/` you can find many recordings of human whole-body postures (i.csv i=1,2,...) and/or body segments poses  (pi.csv i=1,2,...) for several motions. With these you can train and play with your own ProMPs. Have fun!

## Python module

Python module is built using pybind11. Be sure to get it with ```git submodule update --init --recursive``` before using cmake.

Eigen matrices are converted to numpy matrices. Because of the different interal data storage order, the conversion between them implies a performance overhead.

After ```sudo make install```, the module is automatically installed to Python site-lib folder.

In your python code, you can just import the module and use it as in C++. You can find a .py script in the example folder.

```python
import promp
...
file_list = ["t1.csv", "t2.csv"]
dofs = [1,2,3]

trajectory_group = promp.TrajectoryGroup()
trajectory_group.load_csv_trajectories(file_list, dofs)
trajectory_group.normalize_length()

my_promp = promp.Promp(trajectory_group.trajectories(), 20, 0.05)
gen_traj = my_promp.generate_trajectory()
gen_stddev = my_promp.gen_traj_std_dev()
gen_covariance = my_promp.generate_trajectory_covariance()
```

## Acknowledgments

The development of this software is partially supported by [the European Project H2020 An.Dy](http://andy-project.eu/).


