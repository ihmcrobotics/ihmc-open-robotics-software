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

#include <promp/trajectory.hpp>

using namespace promp;

namespace promp::python
{

void pybind_trajectory(pybind11::module& module)
{

    pybind11::class_<Trajectory>(module, "Trajectory")
        .def(pybind11::init<>())
        .def(pybind11::init<const Eigen::MatrixXd&, double>(),
            pybind11::arg("data"),
            pybind11::arg("speed") = 1.0
        )
        .def("dims", &Trajectory::dims)
        .def("timesteps", &Trajectory::timesteps)
        .def("speed", &Trajectory::speed)
        .def("matrix", &Trajectory::matrix, pybind11::return_value_policy::reference_internal)
        .def("sub_trajectory", 
            static_cast< Trajectory (Trajectory::*)(size_t) const>
                (&Trajectory::sub_trajectory) )
        .def("sub_trajectory", 
            static_cast< Trajectory (Trajectory::*)(const std::vector<size_t>&) const>
                (&Trajectory::sub_trajectory) )
        .def("modulate_in_place", &Trajectory::modulate_in_place, 
            pybind11::arg("timesteps"),
            pybind11::arg("fast") = true
        )
        .def("modulate", &Trajectory::modulate,
            pybind11::arg("steps"),
            pybind11::arg("fast") = true
        )
        .def("distance", &Trajectory::distance,
            pybind11::arg("other"),
            pybind11::arg("modulate") = false
        )
        .def("infer_speed", &Trajectory::infer_speed)
        ;

}

} // end namespace python
