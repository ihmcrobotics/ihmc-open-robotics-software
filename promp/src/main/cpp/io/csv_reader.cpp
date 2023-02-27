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

#include "promp/io/csv_reader.hpp"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace promp::io
{

CSVReader::CSVReader(const std::string& filename, char delm, bool skip_header) :
	_filename(filename), 
	_delimeter(delm), 
	_skip_header(skip_header)
{
	//just initialize attributes
}


Eigen::MatrixXd CSVReader::get_data()
{
	std::ifstream file(_filename.c_str()); 
	if (!file.is_open())
		throw std::runtime_error("Error reading file " + _filename);

	std::vector<double> all_values;
	std::string line, token;

	int n_lines = 0;
	int n_elems = 0;

	_names_to_cols.clear();

	bool local_skip_header = _skip_header;
	while (getline(file, line))
	{
		if (local_skip_header)
		{
			int c = 0;
			std::stringstream line_stream(line);
			while(std::getline(line_stream, token, _delimeter))
				_names_to_cols[token] = c++;
			
			local_skip_header = false;
			continue;
		}

		int elements = 0;
		std::stringstream line_stream(line);
		while(std::getline(line_stream, token, _delimeter))
		{
			try
			{ 
				all_values.push_back(std::stod(token));
			}
			catch (...)
			{
				std::stringstream ss;
				ss << "Invalid elements inside " << _filename << " at line " << (n_lines+1)
					<<". Check if all elements are doubles. stod(" << token << ") failed. ";

				throw std::invalid_argument(ss.str());
			}

			++elements;
		}
		
		if(n_elems == 0)
		{
			n_elems = elements;
		}	
		else if(n_elems != elements)
		{
			std::stringstream ss;
			ss << "Invalid elements number " << _filename << " at line " << n_lines + 1
				<< ". Check if lines has the same amount of data. " 
				<< n_elems << " != " << elements << std::endl;
			
			throw std::runtime_error(ss.str());
		}

		++n_lines;
	}
	file.close();

	return Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(all_values.data(), n_lines, n_elems);
}

Eigen::VectorXd CSVReader::get_data_dof(size_t dof, const Eigen::MatrixXd& data)
{
	if (dof < 0 || dof > data.cols())
		throw std::out_of_range("Indexing error in .csv file. dof = columns in .csv file. Indexing starts from 0.");
	
	return data.col(dof);
}

Eigen::MatrixXd CSVReader::get_data_dofs(const std::vector<size_t>& dofs, const Eigen::MatrixXd& data)
{
	size_t i = 0;
	Eigen::MatrixXd selection(data.rows(), dofs.size());
	for(size_t dof : dofs)
	{
		if (dof < 0 || dof > data.cols())
			throw std::out_of_range("Indexing error in .csv file. dofs = columns in .csv file. Indexing starts from 0.");

		selection.col(i++) = data.col(dof);
	}	

	return selection;
}

Eigen::VectorXd CSVReader::get_data_dof(const std::string& dof, const Eigen::MatrixXd& data)
{
	return get_data_dof(_names_to_cols.at(dof), data);
}

Eigen::MatrixXd CSVReader::get_data_dofs(const std::vector<std::string>& dofs, const Eigen::MatrixXd& data)
{
	std::vector<size_t> dofs_idxs;
	for(const auto& d : dofs)
		dofs_idxs.push_back(_names_to_cols.at(d));
	return get_data_dofs(dofs_idxs, data);
}

} // end namespace promp::io
