#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Geometry>
#include "sdf_tools/sdf.hpp"
#include "visualization_msgs/Marker.h"
#include "multiresolution_voxelgrid_manager/multiresolution_voxelgrid.hpp"

#ifndef MULTIRESOLUTION_VOXELGRID_MANAGER_HPP
#define MULTIRESOLUTION_VOXELGRID_MANAGER_HPP

namespace multiresolution_voxelgrid_manager
{
    typedef struct
    {
        double d1;
        double d2;
    } sdf_cell;

    typedef struct
    {
        u_int32_t location[3];
        u_int32_t closest_point[3];
        double distance_square;
        int32_t update_direction;
    } bucket_cell;

    typedef VoxelGrid::VoxelGrid<sdf_cell> DistanceField2D;

    typedef VoxelGrid::VoxelGrid<bucket_cell> DistanceField3D;

    inline double ComputeDistanceSquared(int32_t x1, int32_t y1, int32_t z1, int32_t x2, int32_t y2, int32_t z2)
    {
        int32_t dx = x1 - x2;
        int32_t dy = y1 - y2;
        int32_t dz = z1 - z2;
        return double((dx * dx) + (dy * dy) + (dz * dz));
    }

    enum CELL_STATE {UNKNOWN = 0x00, KNOWN_EMPTY = 0xFE, KNOWN_FILLED = 0xFF, LIKELY_EMPTY = 0xAE, LIKELY_FILLED = 0xAF, PENDING_LIKELY_EMPTY = 0xCE, PENDING_LIKELY_FILLED = 0xCF};

    enum UPDATE_MODE {UPDATE_LOW_RES_ONLY, UPDATE_HIGH_RES_ONLY, UPDATE_BOTH};

    enum UPDATE_TYPE {UPDATE_UNKNOWN_ONLY, UPDATE_LIKELY, UPDATE_ALL};

    enum QUERY_MODE {QUERY_LOW_RES_ONLY, QUERY_HIGH_RES_ONLY, QUERY_HIGH_RES_PREFERED, QUERY_LOW_RES_PREFERED};

    enum QUERY_RESULT {RESULT_LOW_RES_ONLY, RESULT_HIGH_RES_ONLY, RESULT_LOW_RES_KNOWN, RESULT_HIGH_RES_KNOWN, RESULT_BOTH_UNKNOWN};

    class MultiresolutionVoxelGridManager
    {
    protected:

        MultiresolutionVoxelGrid<CELL_STATE> grid_;
        std::string frame_;

        void Build2DDistanceField(DistanceField2D& marked_field);

        inline sdf_cell get_xy(DistanceField2D& partial_field, int64_t x, int64_t y)
        {
            std::pair<sdf_cell, bool> res = partial_field.GetImmutable(x, y, 0);
            if (res.second)
            {
                return res.first;
            }
            else
            {
                sdf_cell empty;
                empty.d1 = INFINITY;
                empty.d2 = INFINITY;
                return empty;
            }
        }

        inline sdf_cell get_xz(DistanceField2D& partial_field, int64_t x, int64_t z)
        {
            std::pair<sdf_cell, bool> res = partial_field.GetImmutable(x, 0, z);
            if (res.second)
            {
                return res.first;
            }
            else
            {
                sdf_cell empty;
                empty.d1 = INFINITY;
                empty.d2 = INFINITY;
                return empty;
            }
        }

        inline sdf_cell get_yz(DistanceField2D& partial_field, int64_t y, int64_t z)
        {
            std::pair<sdf_cell, bool> res = partial_field.GetImmutable(0, y, z);
            if (res.second)
            {
                return res.first;
            }
            else
            {
                sdf_cell empty;
                empty.d1 = INFINITY;
                empty.d2 = INFINITY;
                return empty;
            }
        }

        inline void put_xy(DistanceField2D& partial_field, sdf_cell& cell, int64_t x, int64_t y)
        {
            partial_field.SetValue(x, y, 0, cell);
        }

        inline void put_xz(DistanceField2D& partial_field, sdf_cell& cell, int64_t x, int64_t z)
        {
            partial_field.SetValue(x, 0, z, cell);
        }

        inline void put_yz(DistanceField2D& partial_field, sdf_cell& cell, int64_t y, int64_t z)
        {
            partial_field.SetValue(0, y, z, cell);
        }

        inline double distance_squared(sdf_cell& cell)
        {
            return ((cell.d1 * cell.d1) + (cell.d2 * cell.d2));
        }

        inline void compare_xy(DistanceField2D& partial_field, sdf_cell& cell, int64_t x, int64_t y, int64_t x_offset, int64_t y_offset)
        {
            sdf_cell other = get_xy(partial_field, x + x_offset, y + y_offset);
            other.d1 += x_offset;
            other.d2 += y_offset;
            if (distance_squared(other) < distance_squared(cell))
            {
                cell = other;
            }
        }

        inline void compare_xz(DistanceField2D& partial_field, sdf_cell& cell, int64_t x, int64_t z, int64_t x_offset, int64_t z_offset)
        {
            sdf_cell other = get_xz(partial_field, x + x_offset, z + z_offset);
            other.d1 += x_offset;
            other.d2 += z_offset;
            if (distance_squared(other) < distance_squared(cell))
            {
                cell = other;
            }
        }

        inline void compare_yz(DistanceField2D& partial_field, sdf_cell& cell, int64_t y, int64_t z, int64_t y_offset, int64_t z_offset)
        {
            sdf_cell other = get_yz(partial_field, y + y_offset, z + z_offset);
            other.d1 += y_offset;
            other.d2 += z_offset;
            if (distance_squared(other) < distance_squared(cell))
            {
                cell = other;
            }
        }

        DistanceField3D Build3DDistanceField(std::vector<Eigen::Vector3i>& points, double resolution);

        std::vector<std::vector<std::vector<std::vector<int>>>> MakeNeighborhoods();

        inline int GetDirectionNumber(int dx, int dy, int dz)
        {
            return ((dx + 1) * 9) + ((dy + 1) * 3) + (dz + 1);
        }

        sdf_tools::SignedDistanceField BuildSDF(QUERY_MODE query_mode, double resolution, float OOB_value, bool mark_unknown_as_filled);

    public:

        MultiresolutionVoxelGridManager(Eigen::Affine3d origin_transform, std::string frame, double x_size, double y_size, double z_size, double low_resolution_cell_size, double high_resolution_cell_size, CELL_STATE default_low_res_state, CELL_STATE default_high_res_state);

        MultiresolutionVoxelGridManager() {}

        void AddObservations(std::vector<std::pair<Eigen::Vector3d, CELL_STATE>>& observations, UPDATE_MODE update_mode, UPDATE_TYPE update_type, bool auto_update=true);

        void SweepAndUpdateLowResolutionGrid();

        void SweepAndUpdateHighResolutionGrid();

        void UpdateLowResolutionGridFromHighResolutionGrid();

        void UpdateHighResolutionGridFromLowResolutionGrid();

        visualization_msgs::Marker ExportForDisplay(QUERY_MODE query_mode, float alpha=0.5);

        visualization_msgs::Marker ExportHighResolutionForDisplay(float alpha=0.5);

        visualization_msgs::Marker ExportLowResolutionForDisplay(float alpha=0.5);

        sdf_tools::SignedDistanceField BuildLowResolutionSDF(QUERY_MODE query_mode, float OOB_value, bool mark_unknown_as_filled);

        sdf_tools::SignedDistanceField BuildHighResolutionSDF(QUERY_MODE query_mode, float OOB_value, bool mark_unknown_as_filled);

        inline std::pair<CELL_STATE, bool> QueryLowResolutionGrid(Eigen::Vector3d position)
        {
            return grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z());
        }

        inline std::pair<CELL_STATE, bool> QueryHighResolutionGrid(Eigen::Vector3d position)
        {
            return grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z());
        }

        inline std::pair<CELL_STATE, CELL_STATE> QueryBothResolutionGrids(Eigen::Vector3d position)
        {
            CELL_STATE low_res_val = grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first;
            CELL_STATE high_res_val = grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first;
            return std::pair<CELL_STATE, CELL_STATE>(low_res_val, high_res_val);
        }

        inline std::pair<CELL_STATE, QUERY_RESULT> QueryMultiresolutionGrid(Eigen::Vector3d position, QUERY_MODE query_mode)
        {
            if (query_mode == QUERY_MODE::QUERY_LOW_RES_ONLY)
            {
                return std::pair<CELL_STATE, QUERY_RESULT>(QueryLowResolutionGrid(position).first, QUERY_RESULT::RESULT_LOW_RES_ONLY);
            }
            else if (query_mode == QUERY_MODE::QUERY_HIGH_RES_ONLY)
            {
                return std::pair<CELL_STATE, QUERY_RESULT>(QueryHighResolutionGrid(position).first, QUERY_RESULT::RESULT_HIGH_RES_ONLY);
            }
            else if (query_mode == QUERY_MODE::QUERY_LOW_RES_PREFERED)
            {
                std::pair<CELL_STATE, CELL_STATE> grid_values = QueryBothResolutionGrids(position);
                CELL_STATE low_res_value = grid_values.first;
                CELL_STATE high_res_value = grid_values.second;
                if (low_res_value != CELL_STATE::UNKNOWN)
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(low_res_value, QUERY_RESULT::RESULT_LOW_RES_KNOWN);
                }
                else if (high_res_value != CELL_STATE::UNKNOWN)
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(high_res_value, QUERY_RESULT::RESULT_HIGH_RES_KNOWN);
                }
                else
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(CELL_STATE::UNKNOWN, QUERY_RESULT::RESULT_BOTH_UNKNOWN);
                }
            }
            else if (query_mode == QUERY_MODE::QUERY_HIGH_RES_PREFERED)
            {
                std::pair<CELL_STATE, CELL_STATE> grid_values = QueryBothResolutionGrids(position);
                CELL_STATE low_res_value = grid_values.first;
                CELL_STATE high_res_value = grid_values.second;
                if (high_res_value != CELL_STATE::UNKNOWN)
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(high_res_value, QUERY_RESULT::RESULT_HIGH_RES_KNOWN);
                }
                else if (low_res_value != CELL_STATE::UNKNOWN)
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(low_res_value, QUERY_RESULT::RESULT_LOW_RES_KNOWN);
                }
                else
                {
                    return std::pair<CELL_STATE, QUERY_RESULT>(CELL_STATE::UNKNOWN, QUERY_RESULT::RESULT_BOTH_UNKNOWN);
                }
            }
            else
            {
                throw std::invalid_argument("Invalid query mode");
            }
        }

        inline CELL_STATE DetermineUpdatedValue(std::vector<CELL_STATE> neighbors)
        {
            int unknown_neighbors = 0;
            int empty_neighbors = 0;
            int filled_neighbors = 0;
            for (size_t idx = 0; idx < neighbors.size(); idx++)
            {
                if (neighbors[idx] == CELL_STATE::KNOWN_EMPTY || neighbors[idx] == CELL_STATE::LIKELY_EMPTY)
                {
                    empty_neighbors++;
                }
                else if (neighbors[idx] == CELL_STATE::KNOWN_FILLED || neighbors[idx] == CELL_STATE::LIKELY_FILLED)
                {
                    filled_neighbors++;
                }
                else
                {
                    unknown_neighbors++;
                }
            }
            // If most neighbors are unknown, leave the cell unknown
            if (unknown_neighbors >= filled_neighbors && unknown_neighbors >= empty_neighbors)
            {
                return CELL_STATE::UNKNOWN;
            }
            // If most neighbors are filled, change the cell to filled
            else if (filled_neighbors >= empty_neighbors && filled_neighbors >= unknown_neighbors)
            {
                return CELL_STATE::LIKELY_FILLED;
            }
            // If most neighbors are empty, change the cell to empty
            else if (empty_neighbors >= filled_neighbors && empty_neighbors >= unknown_neighbors)
            {
                return CELL_STATE::LIKELY_EMPTY;
            }
            // If we don't know, leave the cell unknown
            else
            {
                return CELL_STATE::UNKNOWN;
            }
        }

        inline MultiresolutionVoxelGrid<CELL_STATE>& GetGrid()
        {
            return grid_;
        }
    };
}

#endif // MULTIRESOLUTION_VOXELGRID_MANAGER_HPP
