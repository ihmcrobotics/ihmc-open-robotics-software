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

#include <promp/promp.hpp>

namespace promp::python
{

void pybind_promp(pybind11::module& module)
{

    pybind11::class_<ProMP>(module, "ProMP")
        .def(pybind11::init<const std::vector<Trajectory>&, int, double>(),
            pybind11::arg("datas"),
            pybind11::arg("num_bf"),
            pybind11::arg("std_bf")=-1.0)
        .def(pybind11::init<const Eigen::VectorXd&, const Eigen::MatrixXd&, double, int, size_t, double>(),
            pybind11::arg("w"),
            pybind11::arg("cov_w"),
            pybind11::arg("std_bf"),
            pybind11::arg("n_sample"),
            pybind11::arg("dims"),
            pybind11::arg("time_mod") = 1.0
        )
        .def("get_basis_function", &ProMP::get_basis_function, pybind11::return_value_policy::reference_internal)
        .def("generate_basis_function", &ProMP::generate_basis_function)
        .def("compute_phase", static_cast<Eigen::VectorXd(ProMP::*)(size_t) const>(&ProMP::compute_phase) )
        .def("get_phase", &ProMP::get_phase, pybind11::return_value_policy::reference_internal)
        .def("get_weights", &ProMP::get_weights, pybind11::return_value_policy::reference_internal)
        .def("get_covariance", &ProMP::get_covariance, pybind11::return_value_policy::reference_internal)
        .def("get_n_samples", &ProMP::get_n_samples)
        .def("get_traj_length", &ProMP::get_traj_length)
        .def("get_std_bf", &ProMP::get_std_bf)
        .def("get_dims", &ProMP::get_dims)
        // .def("get_mean_demo_time_mod", &ProMP::get_mean_demo_time_mod)
        .def("set_ridge_factor", &ProMP::set_ridge_factor)
        .def("generate_trajectory", static_cast< Eigen::MatrixXd (ProMP::*)() const>(&ProMP::generate_trajectory) )
        .def("generate_trajectory", static_cast< Eigen::MatrixXd (ProMP::*)(size_t) const>(&ProMP::generate_trajectory) )
        .def("generate_trajectory_with_speed", &ProMP::generate_trajectory_with_speed)
        .def("generate_trajectory_at", &ProMP::generate_trajectory_at)
        .def("condition_goal", &ProMP::condition_goal)
        .def("condition_start", &ProMP::condition_start)
        .def("gen_traj_std_dev", &ProMP::gen_traj_std_dev, pybind11::arg("req_num_steps") = 0)
        .def("generate_trajectory_covariance", &ProMP::generate_trajectory_covariance, pybind11::arg("req_num_steps") = 0)
        .def("condition_via_points", &ProMP::condition_via_points)
        .def("condition_via_point", &ProMP::condition_via_point)
        .def("get_upper_weights", &ProMP::get_upper_weights)
        .def("get_lower_weights", &ProMP::get_lower_weights)
        // .def("print", &ProMP::operator<<)
        .def("phase_speed_from_steps", &ProMP::phase_speed_from_steps)
        ;

}

} // end namespace python
