// Copyright 2024 Francisco Miguel Moreno
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
//  Author: Francisco Miguel Moreno

#ifndef GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_
#define GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_

#include <iostream>
#include <vector>
#include <filesystem>

namespace gpe
{

/** Iterate a given directory (non-recursively) and return the path of all CSV files in it. */
std::vector<std::filesystem::path> find_measurement_files(
  const std::filesystem::path & measurements_path)
{
  std::vector<std::filesystem::path> csv_files;
  for (const auto & entry : std::filesystem::directory_iterator(measurements_path)) {
    if (entry.is_regular_file() && entry.path().extension() == ".csv") {
      csv_files.push_back(entry.path());
    }
  }
  return csv_files;
}

}  // namespace gpe

#endif  // GPE_OFFLINE_ESTIMATION__FILE_IO_HPP_
