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

#ifndef PROMP_DATA_SERIALIZER_HPP
#define PROMP_DATA_SERIALIZER_HPP


#include <fstream>
#include <iostream>

#include <promp/promp.hpp>
#include <promp/trajectory.hpp>

namespace promp::io {
    ProMP load_promp(const std::string& path);
    ProMP load_promp(std::istream& stream);

    void save_promp(const std::string& path, const ProMP& promp);

    Trajectory load_trajectory(const std::string& path);
    Trajectory load_trajectory(std::istream& stream);

    void save_trajectory(const std::string& path, const Trajectory& trajectory);

} // end namespace promp::io

#endif // PROMP_DATA_SERIALIZER_HPP