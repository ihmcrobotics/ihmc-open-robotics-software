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

#include <promp/io/serializer.hpp>

#include <fstream>

#include <Eigen/Core>

namespace promp::io
{

ProMP load_promp(const std::string& path)
{
    std::ifstream file(path);
    return load_promp(file);
}

ProMP load_promp(std::istream& stream)
{
    int num_bf;
    int n_samples;
    double std_dev;
    size_t dims;

    stream >> num_bf >> n_samples >> std_dev >> dims;

    double value;
    std::vector<double> serial_data;
    while(stream >> value)
        serial_data.push_back(value);

    size_t weight_size = num_bf * dims;
    Eigen::VectorXd w = Eigen::Map<Eigen::MatrixXd>(serial_data.data(), weight_size, 1);
    Eigen::MatrixXd cov_w = Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(serial_data.data() + weight_size, weight_size, weight_size);

    return ProMP(w, cov_w, std_dev, n_samples, dims);
}

void save_promp(const std::string& path, const ProMP& promp)
{
    Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

    std::ofstream outfile(path);
    outfile << promp.get_weights().size() / promp.get_dims() << std::endl; // num_bf
    outfile << promp.get_n_samples() << std::endl;
    outfile << promp.get_std_bf() << std::endl;
    outfile << promp.get_dims() << std::endl;
    outfile << promp.get_weights().transpose().format(csv_format) << std::endl;
    outfile << promp.get_covariance().format(csv_format) << std::endl;
    outfile.close();
}


Trajectory load_trajectory(const std::string& path)
{
    std::ifstream file(path);
    return load_trajectory(file);
}

Trajectory load_trajectory(std::istream& stream)
{
    size_t dims, timesteps;
    double speed;

    stream >> dims >> timesteps >> speed;

    Eigen::MatrixXd data(timesteps, dims);
    for(int r=0; r < timesteps; ++r)
        for(int c=0; c < dims; ++c)
            stream >> data(r, c);
    
    return Trajectory(data, speed);
}

void save_trajectory(const std::string& path, const Trajectory& trajectory)
{
    Eigen::IOFormat csv_format(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");

    std::ofstream outfile(path);
    outfile << trajectory.dims() << std::endl;
    outfile << trajectory.timesteps() << std::endl;
    outfile << trajectory.speed() << std::endl;
    outfile << trajectory.matrix().format(csv_format) << std::endl;
    outfile.close();   
}

} // end namespace promp::io
