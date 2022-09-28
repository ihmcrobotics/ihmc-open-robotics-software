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

#ifndef PROMP_DATA_CSV_READER
#define PROMP_DATA_CSV_READER

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace promp::io {

    /**
     * @brief Class reading data from .csv files.
     */
    class CSVReader {

    public:
        CSVReader(const std::string& filename, char delm = ',', bool skip_header = false);

        Eigen::MatrixXd get_data();

        static Eigen::VectorXd get_data_dof(size_t dof, const Eigen::MatrixXd& data);

        static Eigen::MatrixXd get_data_dofs(const std::vector<size_t>& dofs, const Eigen::MatrixXd& data);

        Eigen::VectorXd get_data_dof(const std::string& dof, const Eigen::MatrixXd& data);

        Eigen::MatrixXd get_data_dofs(const std::vector<std::string>& dofs, const Eigen::MatrixXd& data);

    private:
        std::string _filename;

        char _delimeter;
        bool _skip_header;

        std::unordered_map<std::string, size_t> _names_to_cols;
    };

} // end namespace promp::io

#endif // PROMP_DATA_CSV_READER
