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

#include <promp/io/serializer.hpp>

namespace promp::python
{

void pybind_serializer(pybind11::module& module)
{
    module.def("load_promp", static_cast<ProMP(*)(const std::string&)>(&io::load_promp) );
    module.def("load_promp", static_cast<ProMP(*)(std::istream&)>(&io::load_promp) );
    module.def("save_promp", &io::save_promp);
    module.def("load_trajectory", static_cast<Trajectory(*)(const std::string&)>(&io::load_trajectory) );
    module.def("load_trajectory", static_cast<Trajectory(*)(std::istream&)>(&io::load_trajectory) );
    module.def("save_trajectory", &io::save_trajectory);
}

} // end namespace python
