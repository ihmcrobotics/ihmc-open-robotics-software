//
// Copyright (c) 2022, INRIA
//
// This file is part of promp.
// promp is free software: you can redistribute it and/or modify it under the terms of 
// the GNU Lesser General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.
// promp is distributed in the hope that it will be useful, but WITHOUT ANY 
// WARRANTY; without even the implied warranty of MERCHANTABILITY or 
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for 
// more details.
// You should have received a copy of the GNU Lesser General Public License along with 
// promp. If not, see <https://www.gnu.org/licenses/>. 
//

#ifndef PROMP_PYPROMP_HPP
#define PROMP_PYPROMP_HPP

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>

namespace promp::python
{
    void pybind_trajectory(pybind11::module& module);
    void pybind_csv_reader(pybind11::module& module);
    void pybind_trajectory_group(pybind11::module& module);
    void pybind_promp(pybind11::module& module);
    void pybind_serializer(pybind11::module& module);
}

#endif // PROMP_PYPROMP_HPP
