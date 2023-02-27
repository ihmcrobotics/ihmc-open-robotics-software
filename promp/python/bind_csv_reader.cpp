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

#include <promp/io/csv_reader.hpp>

using namespace promp::io;

namespace promp::python
{

void pybind_csv_reader(pybind11::module& module)
{

    pybind11::class_<CSVReader>(module, "CSVReader")
        .def(pybind11::init<const std::string&, char, bool>(),
            pybind11::arg("filename"),
            pybind11::arg("delm") = ',',
            pybind11::arg("skip_header") = false
        )
        .def("get_data", &CSVReader::get_data)
        //.def_static("get_data_dof", 
        //    static_cast< Eigen::VectorXd (*)(size_t, const Eigen::MatrixXd&)>
        //        (&CSVReader::get_data_dof) )
        .def("get_data_dof", 
            static_cast< Eigen::VectorXd (CSVReader::*)(const std::string&, const Eigen::MatrixXd&)>(&CSVReader::get_data_dof) )
        //.def_static("get_data_dofs", 
        //    static_cast< Eigen::MatrixXd (*)(const std::vector<size_t>&, const Eigen::MatrixXd&)>
        //        (&CSVReader::get_data_dofs) )
        .def("get_data_dofs", 
            static_cast< Eigen::MatrixXd (CSVReader::*)(const std::vector<std::string>&, const Eigen::MatrixXd&)>(&CSVReader::get_data_dofs) )
        ;

}

} // end namespace python
