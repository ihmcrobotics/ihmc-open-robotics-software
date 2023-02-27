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

#include "pypromp.hpp"

#include <promp/trajectory_group.hpp>

namespace promp::python
{

void pybind_trajectory_group(pybind11::module& module)
{

    pybind11::class_<TrajectoryGroup>(module, "TrajectoryGroup")
        .def(pybind11::init<>())
        .def("load_trajectories", &TrajectoryGroup::load_trajectories)
        .def("load_csv_trajectories", 
            static_cast< void (TrajectoryGroup::*)
                (const std::vector<std::string>&, const std::vector<size_t>& index, char, bool)>
            (&TrajectoryGroup::load_csv_trajectories),
            pybind11::arg("files"),
            pybind11::arg("index"),
            pybind11::arg("sep") = ',',
            pybind11::arg("skip_header") = false)
        .def("load_csv_trajectories", 
            static_cast< void (TrajectoryGroup::*)
                (const std::vector<std::string>&, const std::vector<std::string>& index, char)>
            (&TrajectoryGroup::load_csv_trajectories),
            pybind11::arg("files"),
            pybind11::arg("index"),
            pybind11::arg("sep") = ',')
        .def("normalize_length",  static_cast<size_t (TrajectoryGroup::*)()>(&TrajectoryGroup::normalize_length) )
        .def("normalize_length",  static_cast<void (TrajectoryGroup::*)(size_t)>(&TrajectoryGroup::normalize_length) )
        .def("standardize_dims", &TrajectoryGroup::standardize_dims)
        .def("trajectories", &TrajectoryGroup::trajectories, pybind11::return_value_policy::reference_internal)
        ;

}

} // end namespace python
