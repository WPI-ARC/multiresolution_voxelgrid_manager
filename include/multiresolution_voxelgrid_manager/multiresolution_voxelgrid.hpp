#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include "arc_utilities/voxel_grid.hpp"

#ifndef MULTIRESOLUTION_VOXELGRID_HPP
#define MULTIRESOLUTION_VOXELGRID_HPP

namespace multiresolution_voxelgrid_manager
{
    template <typename T>
    class MultiresolutionVoxelGrid
    {
    protected:

        VoxelGrid::VoxelGrid<T> low_resolution_grid_;
        VoxelGrid::VoxelGrid<T> high_resolution_grid_;

    public:

        MultiresolutionVoxelGrid(Eigen::Affine3d origin_transform, double x_size, double y_size, double z_size, double low_resolution_cell_size, double high_resolution_cell_size, T default_low_resolution_value, T default_high_resolution_value)
        {
            VoxelGrid::VoxelGrid<T> new_low_res_grid(origin_transform, low_resolution_cell_size, x_size, y_size, z_size, default_low_resolution_value);
            low_resolution_grid_ = new_low_res_grid;
            VoxelGrid::VoxelGrid<T> new_high_res_grid(origin_transform, high_resolution_cell_size, x_size, y_size, z_size, default_high_resolution_value);
            high_resolution_grid_ = new_high_res_grid;
        }

        MultiresolutionVoxelGrid() {}

        inline bool UpdateLowResolutionGrid(double x, double y, double z, T value)
        {
            return low_resolution_grid_.SetValue(x, y, z, value);
        }

        inline bool UpdateHighResolutionGrid(double x, double y, double z, T value)
        {
            return high_resolution_grid_.SetValue(x, y, z, value);
        }

        inline bool UpdateLowResolutionGrid(int64_t x_index, int64_t y_index, int64_t z_index, T value)
        {
            return low_resolution_grid_.SetValue(x_index, y_index, z_index, value);
        }

        inline bool UpdateHighResolutionGrid(int64_t x_index, int64_t y_index, int64_t z_index, T value)
        {
            return high_resolution_grid_.SetValue(x_index, y_index, z_index, value);
        }

        inline std::pair<T&, bool> CheckLowResolutionGrid(double x, double y, double z)
        {
            return low_resolution_grid_.GetMutable(x, y, z);
        }

        inline std::pair<T&, bool> CheckHighResolutionGrid(double x, double y, double z)
        {
            return high_resolution_grid_.GetMutable(x, y, z);
        }

        inline std::pair<T&, bool> CheckLowResolutionGrid(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return low_resolution_grid_.GetMutable(x_index, y_index, z_index);
        }

        inline std::pair<T&, bool> CheckHighResolutionGrid(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return high_resolution_grid_.GetMutable(x_index, y_index, z_index);
        }

        std::vector<T> CheckLowResolutionGridNeighbors(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            std::vector<T> neighbors;
            // Get each of the neighbors, and add them to the vector if they're valid
            std::pair<T, bool> n1 = low_resolution_grid_.GetImmutable(x_index - 1, y_index - 1, z_index - 1);
            if (n1.second)
            {
                neighbors.push_back(n1.first);
            }
            std::pair<T, bool> n2 = low_resolution_grid_.GetImmutable(x_index - 1, y_index - 1, z_index + 1);
            if (n2.second)
            {
                neighbors.push_back(n2.first);
            }
            std::pair<T, bool> n3 = low_resolution_grid_.GetImmutable(x_index - 1, y_index + 1, z_index - 1);
            if (n3.second)
            {
                neighbors.push_back(n3.first);
            }
            std::pair<T, bool> n4 = low_resolution_grid_.GetImmutable(x_index - 1, y_index + 1, z_index + 1);
            if (n4.second)
            {
                neighbors.push_back(n4.first);
            }
            std::pair<T, bool> n5 = low_resolution_grid_.GetImmutable(x_index + 1, y_index - 1, z_index - 1);
            if (n5.second)
            {
                neighbors.push_back(n5.first);
            }
            std::pair<T, bool> n6 = low_resolution_grid_.GetImmutable(x_index + 1, y_index - 1, z_index + 1);
            if (n6.second)
            {
                neighbors.push_back(n6.first);
            }
            std::pair<T, bool> n7 = low_resolution_grid_.GetImmutable(x_index + 1, y_index + 1, z_index - 1);
            if (n7.second)
            {
                neighbors.push_back(n7.first);
            }
            std::pair<T, bool> n8 = low_resolution_grid_.GetImmutable(x_index + 1, y_index + 1, z_index + 1);
            if (n8.second)
            {
                neighbors.push_back(n8.first);
            }
            return neighbors;
        }

        std::vector<T> CheckHighResolutionGridNeighbors(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            std::vector<T> neighbors;
            // Get each of the neighbors, and add them to the vector if they're valid
            std::pair<T, bool> n1 = high_resolution_grid_.GetImmutable(x_index - 1, y_index - 1, z_index - 1);
            if (n1.second)
            {
                neighbors.push_back(n1.first);
            }
            std::pair<T, bool> n2 = high_resolution_grid_.GetImmutable(x_index - 1, y_index - 1, z_index + 1);
            if (n2.second)
            {
                neighbors.push_back(n2.first);
            }
            std::pair<T, bool> n3 = high_resolution_grid_.GetImmutable(x_index - 1, y_index + 1, z_index - 1);
            if (n3.second)
            {
                neighbors.push_back(n3.first);
            }
            std::pair<T, bool> n4 = high_resolution_grid_.GetImmutable(x_index - 1, y_index + 1, z_index + 1);
            if (n4.second)
            {
                neighbors.push_back(n4.first);
            }
            std::pair<T, bool> n5 = high_resolution_grid_.GetImmutable(x_index + 1, y_index - 1, z_index - 1);
            if (n5.second)
            {
                neighbors.push_back(n5.first);
            }
            std::pair<T, bool> n6 = high_resolution_grid_.GetImmutable(x_index + 1, y_index - 1, z_index + 1);
            if (n6.second)
            {
                neighbors.push_back(n6.first);
            }
            std::pair<T, bool> n7 = high_resolution_grid_.GetImmutable(x_index + 1, y_index + 1, z_index - 1);
            if (n7.second)
            {
                neighbors.push_back(n7.first);
            }
            std::pair<T, bool> n8 = high_resolution_grid_.GetImmutable(x_index + 1, y_index + 1, z_index + 1);
            if (n8.second)
            {
                neighbors.push_back(n8.first);
            }
            return neighbors;
        }

        inline Eigen::Affine3d GetOriginTransform()
        {
            return low_resolution_grid_.GetOriginTransform();
        }

        inline double GetXSize()
        {
            return low_resolution_grid_.GetXSize();
        }

        inline double GetYSize()
        {
            return low_resolution_grid_.GetYSize();
        }

        inline double GetZSize()
        {
            return low_resolution_grid_.GetZSize();
        }

        inline int64_t GetLowResolutionNumXCells()
        {
            return low_resolution_grid_.GetNumXCells();
        }

        inline int64_t GetLowResolutionNumYCells()
        {
            return low_resolution_grid_.GetNumYCells();
        }

        inline int64_t GetLowResolutionNumZCells()
        {
            return low_resolution_grid_.GetNumZCells();
        }

        inline int64_t GetHighResolutionNumXCells()
        {
            return high_resolution_grid_.GetNumXCells();
        }

        inline int64_t GetHighResolutionNumYCells()
        {
            return high_resolution_grid_.GetNumYCells();
        }

        inline int64_t GetHighResolutionNumZCells()
        {
            return high_resolution_grid_.GetNumZCells();
        }

        inline double GetLowResolution()
        {
            return low_resolution_grid_.GetCellSizes()[0];
        }

        inline double GetHighResolution()
        {
            return high_resolution_grid_.GetCellSizes()[0];
        }

        inline T GetDefaultLowResolutionValue()
        {
            return low_resolution_grid_.GetDefaultValue();
        }

        inline T GetDefaultHighResolutionValue()
        {
            return high_resolution_grid_.GetDefaultValue();
        }

        inline std::vector<double> LowResolutionGridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return low_resolution_grid_.GridIndexToLocation(x_index, y_index, z_index);
        }

        inline std::vector<double> HighResolutionGridIndexToLocation(int64_t x_index, int64_t y_index, int64_t z_index)
        {
            return high_resolution_grid_.GridIndexToLocation(x_index, y_index, z_index);
        }

        void ResetLowResolutionGrid()
        {
            low_resolution_grid_.ResetWithDefault();
        }

        void ResetHighResolutionGrid()
        {
            high_resolution_grid_.ResetWithDefault();
        }

        void ResetGrids()
        {
            ResetLowResolutionGrid();
            ResetHighResolutionGrid();
        }

    };

}

#endif // MULTIRESOLUTION_VOXELGRID_HPP
