#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <ros/ros.h>
#include "multiresolution_voxelgrid_manager/multiresolution_voxelgrid.hpp"
#include "multiresolution_voxelgrid_manager/multiresolution_voxelgrid_manager.hpp"

using namespace multiresolution_voxelgrid_manager;

MultiresolutionVoxelGridManager::MultiresolutionVoxelGridManager(Eigen::Affine3d origin_transform, std::string frame, double x_size, double y_size, double z_size, double low_resolution_cell_size, double high_resolution_cell_size, CELL_STATE default_low_res_state, CELL_STATE default_high_res_state)
{
    MultiresolutionVoxelGrid<CELL_STATE> new_grid(origin_transform, x_size, y_size, z_size, low_resolution_cell_size, high_resolution_cell_size, default_low_res_state, default_high_res_state);
    grid_ = new_grid;
    frame_ = frame;
}

void MultiresolutionVoxelGridManager::AddObservations(std::vector<std::pair<Eigen::Vector3d, CELL_STATE>>& observations, UPDATE_MODE update_mode, UPDATE_TYPE update_type, bool auto_update)
{
    if (update_type == UPDATE_TYPE::UPDATE_ALL)
    {
        if (update_mode == UPDATE_MODE::UPDATE_BOTH)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
                grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_LOW_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_HIGH_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
            }
            if (auto_update)
            {
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else
        {
            throw std::invalid_argument("Invalid update mode");
        }
    }
    else if (update_type == UPDATE_TYPE::UPDATE_LIKELY)
    {
        if (update_mode == UPDATE_MODE::UPDATE_BOTH)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_EMPTY && grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_FILLED)
                {
                    grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
                if (grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_EMPTY && grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_FILLED)
                {
                    grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_LOW_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_EMPTY && grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_FILLED)
                {
                    grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_HIGH_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_EMPTY && grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first != CELL_STATE::KNOWN_FILLED)
                {
                    grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else
        {
            throw std::invalid_argument("Invalid update mode");
        }
    }
    else if (update_type == UPDATE_TYPE::UPDATE_UNKNOWN_ONLY)
    {
        if (update_mode == UPDATE_MODE::UPDATE_BOTH)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first == CELL_STATE::UNKNOWN)
                {
                    grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
                if (grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first == CELL_STATE::UNKNOWN)
                {
                    grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_LOW_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckLowResolutionGrid(position.x(), position.y(), position.z()).first == CELL_STATE::UNKNOWN)
                {
                    grid_.UpdateLowResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateLowResolutionGrid();
            }
        }
        else if (update_mode == UPDATE_MODE::UPDATE_HIGH_RES_ONLY)
        {
            for (size_t idx = 0; idx < observations.size(); idx++)
            {
                Eigen::Vector3d position = observations[idx].first;
                CELL_STATE observation = observations[idx].second;
                if (grid_.CheckHighResolutionGrid(position.x(), position.y(), position.z()).first == CELL_STATE::UNKNOWN)
                {
                    grid_.UpdateHighResolutionGrid(position.x(), position.y(), position.z(), observation);
                }
            }
            if (auto_update)
            {
                SweepAndUpdateHighResolutionGrid();
            }
        }
        else
        {
            throw std::invalid_argument("Invalid update mode");
        }
    }
    else
    {
        throw std::invalid_argument("Invalid update type");
    }
}

void MultiresolutionVoxelGridManager::SweepAndUpdateLowResolutionGrid()
{
    // Sweep through and mark pending updates
    for (int64_t x_index = 0; x_index < grid_.GetLowResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetLowResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetLowResolutionNumZCells(); z_index++)
            {
                CELL_STATE cell_value = grid_.CheckLowResolutionGrid(x_index, y_index, z_index).first;
                // We only update the cells that are currently unknown
                if (cell_value == CELL_STATE::UNKNOWN)
                {
                    // Get the neighboring cell values
                    std::vector<CELL_STATE> neighbors = grid_.CheckLowResolutionGridNeighbors(x_index, y_index, z_index);
                    // Determine the correct value to update the cell
                    CELL_STATE new_value = DetermineUpdatedValue(neighbors);
                    // Set the new value as a pending change
                    if (new_value == CELL_STATE::LIKELY_EMPTY)
                    {
                        grid_.UpdateLowResolutionGrid(x_index, y_index, z_index, CELL_STATE::PENDING_LIKELY_EMPTY);
                    }
                    else if (new_value == CELL_STATE::LIKELY_FILLED)
                    {
                        grid_.UpdateLowResolutionGrid(x_index, y_index, z_index, CELL_STATE::PENDING_LIKELY_FILLED);
                    }
                }
            }
        }
    }
    // Sweep through and apply pending updates
    for (int64_t x_index = 0; x_index < grid_.GetLowResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetLowResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetLowResolutionNumZCells(); z_index++)
            {
                CELL_STATE cell_value = grid_.CheckLowResolutionGrid(x_index, y_index, z_index).first;
                // We only update the cells that are pending
                if (cell_value == CELL_STATE::PENDING_LIKELY_EMPTY)
                {
                    grid_.UpdateLowResolutionGrid(x_index, y_index, z_index, CELL_STATE::LIKELY_EMPTY);
                }
                else if (cell_value == CELL_STATE::PENDING_LIKELY_FILLED)
                {
                    grid_.UpdateLowResolutionGrid(x_index, y_index, z_index, CELL_STATE::LIKELY_FILLED);
                }
            }
        }
    }
}

void MultiresolutionVoxelGridManager::SweepAndUpdateHighResolutionGrid()
{
    // Sweep through and mark pending updates
    for (int64_t x_index = 0; x_index < grid_.GetHighResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetHighResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetHighResolutionNumZCells(); z_index++)
            {
                CELL_STATE cell_value = grid_.CheckHighResolutionGrid(x_index, y_index, z_index).first;
                // We only update the cells that are currently unknown
                if (cell_value == CELL_STATE::UNKNOWN)
                {
                    // Get the neighboring cell values
                    std::vector<CELL_STATE> neighbors = grid_.CheckHighResolutionGridNeighbors(x_index, y_index, z_index);
                    // Determine the correct value to update the cell
                    CELL_STATE new_value = DetermineUpdatedValue(neighbors);
                    // Set the new value as a pending change
                    if (new_value == CELL_STATE::LIKELY_EMPTY)
                    {
                        grid_.UpdateHighResolutionGrid(x_index, y_index, z_index, CELL_STATE::PENDING_LIKELY_EMPTY);
                    }
                    else if (new_value == CELL_STATE::LIKELY_FILLED)
                    {
                        grid_.UpdateHighResolutionGrid(x_index, y_index, z_index, CELL_STATE::PENDING_LIKELY_FILLED);
                    }
                }
            }
        }
    }
    // Sweep through and apply pending updates
    for (int64_t x_index = 0; x_index < grid_.GetHighResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetHighResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetHighResolutionNumZCells(); z_index++)
            {
                CELL_STATE cell_value = grid_.CheckHighResolutionGrid(x_index, y_index, z_index).first;
                // We only update the cells that are pending
                if (cell_value == CELL_STATE::PENDING_LIKELY_EMPTY)
                {
                    grid_.UpdateHighResolutionGrid(x_index, y_index, z_index, CELL_STATE::LIKELY_EMPTY);
                }
                else if (cell_value == CELL_STATE::PENDING_LIKELY_FILLED)
                {
                    grid_.UpdateHighResolutionGrid(x_index, y_index, z_index, CELL_STATE::LIKELY_FILLED);
                }
            }
        }
    }
}

void MultiresolutionVoxelGridManager::UpdateHighResolutionGridFromLowResolutionGrid()
{
    // Loop through all the grid cells
    for (int64_t x_index = 0; x_index < grid_.GetHighResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetHighResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetHighResolutionNumZCells(); z_index++)
            {
                // Get the real (X,Y,Z) location corresponding to the index
                std::vector<double> real_pos = grid_.HighResolutionGridIndexToLocation(x_index, y_index, z_index);
                if (real_pos.size() == 3)
                {
                    // Lookup the corresponding value in the low resolution grid
                    std::pair<CELL_STATE, bool> low_res_val = grid_.CheckLowResolutionGrid(real_pos[0], real_pos[1], real_pos[2]);
                    if (low_res_val.second)
                    {
                        CELL_STATE new_state = low_res_val.first;
                        // Update the high resolution grid with the new value
                        grid_.UpdateHighResolutionGrid(x_index, y_index, z_index, new_state);
                    }
                }
            }
        }
    }
}

void MultiresolutionVoxelGridManager::UpdateLowResolutionGridFromHighResolutionGrid()
{
    // Loop through all the grid cells
    for (int64_t x_index = 0; x_index < grid_.GetLowResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetLowResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetLowResolutionNumZCells(); z_index++)
            {
                // Get the real (X,Y,Z) location corresponding to the index
                std::vector<double> real_pos = grid_.LowResolutionGridIndexToLocation(x_index, y_index, z_index);
                if (real_pos.size() == 3)
                {
                    // Look up all the corresponding values in the high resolution grid
                    // Lookup the corresponding value in the low resolution grid
                    std::pair<CELL_STATE, bool> high_res_val = grid_.CheckHighResolutionGrid(real_pos[0], real_pos[1], real_pos[2]);
                    if (high_res_val.second)
                    {
                        CELL_STATE new_state = high_res_val.first;
                        // Update the high resolution grid with the new value
                        grid_.UpdateLowResolutionGrid(x_index, y_index, z_index, new_state);
                    }
                }
            }
        }
    }
}

void MultiresolutionVoxelGridManager::Build2DDistanceField(DistanceField2D& marked_field)
{
    // Convert the marked field into a real distance field using 8SSEDT
    // First, determine which dimension we're ignoring
    // Z-axis can be ignored
    if (marked_field.GetNumXCells() > 1 && marked_field.GetNumYCells() > 1 && marked_field.GetNumZCells() == 1)
    {
        // Pass 1.1.0
        for (int64_t y = 0; y < marked_field.GetNumYCells(); y++)
        {
            // Pass 1.1.1
            for (int64_t x = 0; x < marked_field.GetNumXCells(); x++)
            {
                // Get value from grid
                sdf_cell cell = get_xy(marked_field, x, y);
                // Do work
                compare_xy(marked_field, cell, x, y, -1, 0);
                compare_xy(marked_field, cell, x, y, 0, -1);
                compare_xy(marked_field, cell, x, y, -1, -1);
                compare_xy(marked_field, cell, x, y, 1, -1);
                // Store back into grid
                put_xy(marked_field, cell, x, y);
            }
            // Pass 1.1.2
            for (int64_t x = (marked_field.GetNumXCells() - 1); x >= 0; x--)
            {
                // Get value from grid
                sdf_cell cell = get_xy(marked_field, x, y);
                // Do work
                compare_xy(marked_field, cell, x, y, 1, 0);
                // Store back into grid
                put_xy(marked_field, cell, x, y);
            }
        }
        // Pass 1.2.0
        for (int64_t y = (marked_field.GetNumYCells() - 1); y >= 0; y--)
        {
            // Pass 1.1.1
            for (int64_t x = (marked_field.GetNumXCells() - 1); x >= 0; x--)
            {
                // Get value from grid
                sdf_cell cell = get_xy(marked_field, x, y);
                // Do work
                compare_xy(marked_field, cell, x, y, 1, 0);
                compare_xy(marked_field, cell, x, y, 0, 1);
                compare_xy(marked_field, cell, x, y, -1, 1);
                compare_xy(marked_field, cell, x, y, 1, 1);
                // Store back into grid
                put_xy(marked_field, cell, x, y);
            }
            // Pass 1.1.2
            for (int64_t x = 0; x < marked_field.GetNumXCells(); x++)
            {
                // Get value from grid
                sdf_cell cell = get_xy(marked_field, x, y);
                // Do work
                compare_xy(marked_field, cell, x, y, -1, 0);
                // Store back into grid
                put_xy(marked_field, cell, x, y);
            }
        }
    }
    // Y-axis can be ignored
    else if (marked_field.GetNumXCells() > 1 && marked_field.GetNumYCells() == 1 && marked_field.GetNumZCells() > 1)
    {
        // Pass 1.1.0
        for (int64_t z = 0; z < marked_field.GetNumZCells(); z++)
        {
            // Pass 1.1.1
            for (int64_t x = 0; x < marked_field.GetNumXCells(); x++)
            {
                // Get value from grid
                sdf_cell cell = get_xz(marked_field, x, z);
                // Do work
                compare_xz(marked_field, cell, x, z, -1, 0);
                compare_xz(marked_field, cell, x, z, 0, -1);
                compare_xz(marked_field, cell, x, z, -1, -1);
                compare_xz(marked_field, cell, x, z, 1, -1);
                // Store back into grid
                put_xz(marked_field, cell, x, z);
            }
            // Pass 1.1.2
            for (int64_t x = (marked_field.GetNumXCells() - 1); x >= 0; x--)
            {
                // Get value from grid
                sdf_cell cell = get_xz(marked_field, x, z);
                // Do work
                compare_xz(marked_field, cell, x, z, 1, 0);
                // Store back into grid
                put_xz(marked_field, cell, x, z);
            }
        }
        // Pass 1.2.0
        for (int64_t z = (marked_field.GetNumZCells() - 1); z >= 0; z--)
        {
            // Pass 1.1.1
            for (int64_t x = (marked_field.GetNumXCells() - 1); x >= 0; x--)
            {
                // Get value from grid
                sdf_cell cell = get_xz(marked_field, x, z);
                // Do work
                compare_xz(marked_field, cell, x, z, 1, 0);
                compare_xz(marked_field, cell, x, z, 0, 1);
                compare_xz(marked_field, cell, x, z, -1, 1);
                compare_xz(marked_field, cell, x, z, 1, 1);
                // Store back into grid
                put_xz(marked_field, cell, x, z);
            }
            // Pass 1.1.2
            for (int64_t x = 0; x < marked_field.GetNumXCells(); x++)
            {
                // Get value from grid
                sdf_cell cell = get_xz(marked_field, x, z);
                // Do work
                compare_xz(marked_field, cell, x, z, -1, 0);
                // Store back into grid
                put_xz(marked_field, cell, x, z);
            }
        }
    }
    // X-axis can be ignored
    else if (marked_field.GetNumXCells() == 1 && marked_field.GetNumYCells() > 1 && marked_field.GetNumZCells() > 1)
    {
        // Pass 1.1.0
        for (int64_t z = 0; z < marked_field.GetNumZCells(); z++)
        {
            // Pass 1.1.1
            for (int64_t y = 0; y < marked_field.GetNumYCells(); y++)
            {
                // Get value from grid
                sdf_cell cell = get_yz(marked_field, y, z);
                // Do work
                compare_yz(marked_field, cell, y, z, -1, 0);
                compare_yz(marked_field, cell, y, z, 0, -1);
                compare_yz(marked_field, cell, y, z, -1, -1);
                compare_yz(marked_field, cell, y, z, 1, -1);
                // Store back into grid
                put_yz(marked_field, cell, y, z);
            }
            // Pass 1.1.2
            for (int64_t y = (marked_field.GetNumYCells() - 1); y >= 0; y--)
            {
                // Get value from grid
                sdf_cell cell = get_yz(marked_field, y, z);
                // Do work
                compare_yz(marked_field, cell, y, z, 1, 0);
                // Store back into grid
                put_yz(marked_field, cell, y, z);
            }
        }
        // Pass 1.2.0
        for (int64_t z = (marked_field.GetNumZCells() - 1); z >= 0; z--)
        {
            // Pass 1.1.1
            for (int64_t y = (marked_field.GetNumYCells() - 1); y >= 0; y--)
            {
                // Get value from grid
                sdf_cell cell = get_yz(marked_field, y, z);
                // Do work
                compare_yz(marked_field, cell, y, z, 1, 0);
                compare_yz(marked_field, cell, y, z, 0, 1);
                compare_yz(marked_field, cell, y, z, -1, 1);
                compare_yz(marked_field, cell, y, z, 1, 1);
                // Store back into grid
                put_yz(marked_field, cell, y, z);
            }
            // Pass 1.1.2
            for (int64_t y = 0; y < marked_field.GetNumYCells(); y++)
            {
                // Get value from grid
                sdf_cell cell = get_yz(marked_field, y, z);
                // Do work
                compare_yz(marked_field, cell, y, z, -1, 0);
                // Store back into grid
                put_yz(marked_field, cell, y, z);
            }
        }
    }
    // Invalid field
    else
    {
        throw std::invalid_argument("Marked distance field is not 2D - cannot process with 8SSEDT");
    }
}

DistanceField3D MultiresolutionVoxelGridManager::Build3DDistanceField(std::vector<Eigen::Vector3i>& points, double resolution)
{
    // Make the DistanceField3D container
    bucket_cell default_cell;
    default_cell.distance_square = INFINITY;
    DistanceField3D distance_field(grid_.GetOriginTransform(), resolution, grid_.GetXSize(), grid_.GetYSize(), grid_.GetZSize(), default_cell);
    // Compute maximum distance square
    long max_distance_square = (distance_field.GetNumXCells() * distance_field.GetNumXCells()) + (distance_field.GetNumYCells() * distance_field.GetNumYCells()) + (distance_field.GetNumZCells() * distance_field.GetNumZCells());
    // Make bucket queue
    std::vector<std::vector<bucket_cell>> bucket_queue(max_distance_square + 1);
    bucket_queue[0].reserve(points.size());
    // Set initial update direction
    int initial_update_direction = GetDirectionNumber(0, 0, 0);
    // Mark all points with distance zero and add to the bucket queue
    for (size_t index = 0; index < points.size(); index++)
    {
        std::pair<bucket_cell&, bool> query = distance_field.GetMutable((int64_t)points[index].x(), (int64_t)points[index].y(), (int64_t)points[index].z());
        if (query.second)
        {
            query.first.location[0] = points[index].x();
            query.first.location[1] = points[index].y();
            query.first.location[2] = points[index].z();
            query.first.closest_point[0] = points[index].x();
            query.first.closest_point[1] = points[index].y();
            query.first.closest_point[2] = points[index].z();
            query.first.distance_square = 0.0;
            query.first.update_direction = initial_update_direction;
            bucket_queue[0].push_back(query.first);
        }
        // If the point is outside the bounds of the SDF, skip
        else
        {
            continue;
        }
    }
    // Process the bucket queue
    std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods = MakeNeighborhoods();
    for (size_t bq_idx = 0; bq_idx < bucket_queue.size(); bq_idx++)
    {
        std::vector<bucket_cell>::iterator queue_itr = bucket_queue[bq_idx].begin();
        while (queue_itr != bucket_queue[bq_idx].end())
        {
            // Get the current location
            bucket_cell& cur_cell = *queue_itr;
            double x = cur_cell.location[0];
            double y = cur_cell.location[1];
            double z = cur_cell.location[2];
            // Pick the update direction
            int D = bq_idx;
            if (D > 1)
            {
                D = 1;
            }
            // Make sure the update direction is valid
            if (cur_cell.update_direction < 0 || cur_cell.update_direction > 26)
            {
                ++queue_itr;
                continue;
            }
            // Get the current neighborhood list
            std::vector<std::vector<int>>& neighborhood = neighborhoods[D][cur_cell.update_direction];
            // Update the distance from the neighboring cells
            for (size_t nh_idx = 0; nh_idx < neighborhood.size(); nh_idx++)
            {
                // Get the direction to check
                int dx = neighborhood[nh_idx][0];
                int dy = neighborhood[nh_idx][1];
                int dz = neighborhood[nh_idx][2];
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;
                std::pair<bucket_cell&, bool> neighbor_query = distance_field.GetMutable((int64_t)nx, (int64_t)ny, (int64_t)nz);
                if (!neighbor_query.second)
                {
                    // "Neighbor" is outside the bounds of the SDF
                    continue;
                }
                // Update the neighbor's distance based on the current
                int new_distance_square = ComputeDistanceSquared(nx, ny, nz, cur_cell.closest_point[0], cur_cell.closest_point[1], cur_cell.closest_point[2]);
                if (new_distance_square > max_distance_square)
                {
                    // Skip these cases
                    continue;
                }
                if (new_distance_square < neighbor_query.first.distance_square)
                {
                    // If the distance is better, time to update the neighbor
                    neighbor_query.first.distance_square = new_distance_square;
                    neighbor_query.first.closest_point[0] = cur_cell.closest_point[0];
                    neighbor_query.first.closest_point[1] = cur_cell.closest_point[1];
                    neighbor_query.first.closest_point[2] = cur_cell.closest_point[2];
                    neighbor_query.first.location[0] = nx;
                    neighbor_query.first.location[1] = ny;
                    neighbor_query.first.location[2] = nz;
                    neighbor_query.first.update_direction = GetDirectionNumber(dx, dy, dz);
                    // Add the neighbor into the bucket queue
                    bucket_queue[new_distance_square].push_back(neighbor_query.first);
                }
            }
            // Increment the queue iterator
            ++queue_itr;
        }
        // Clear the current queue now that we're done with it
        bucket_queue[bq_idx].clear();
    }
    return distance_field;
}

std::vector<std::vector<std::vector<std::vector<int>>>> MultiresolutionVoxelGridManager::MakeNeighborhoods()
{
    std::vector<std::vector<std::vector<std::vector<int>>>> neighborhoods;
    neighborhoods.resize(2);
    for (size_t n = 0; n < neighborhoods.size(); n++)
    {
        neighborhoods[n].resize(27);
        // Loop through the source directions
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    int direction_number = GetDirectionNumber(dx, dy, dz);
                    // Loop through the target directions
                    for (int tdx = -1; tdx <= 1; tdx++)
                    {
                        for (int tdy = -1; tdy <= 1; tdy++)
                        {
                            for (int tdz = -1; tdz <= 1; tdz++)
                            {
                                if (tdx == 0 && tdy == 0 && tdz == 0)
                                {
                                    continue;
                                }
                                if (n >= 1)
                                {
                                    if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1)
                                    {
                                        continue;
                                    }
                                    if ((dx * tdx) < 0 || (dy * tdy) < 0 || (dz * tdz) < 0)
                                    {
                                        continue;
                                    }
                                }
                                std::vector<int> new_point;
                                new_point.resize(3);
                                new_point[0] = tdx;
                                new_point[1] = tdy;
                                new_point[2] = tdz;
                                neighborhoods[n][direction_number].push_back(new_point);
                            }
                        }
                    }
                }
            }
        }
    }
    return neighborhoods;
}

sdf_tools::SignedDistanceField MultiresolutionVoxelGridManager::BuildSDF(QUERY_MODE query_mode, double resolution, float OOB_value, bool mark_unknown_as_filled)
{
    // Make the SDF container
    sdf_tools::SignedDistanceField sdf(grid_.GetOriginTransform(), frame_, resolution, grid_.GetXSize(), grid_.GetYSize(), grid_.GetZSize(), OOB_value);
    // Determine if the current Grid is only 2D
    if (sdf.GetNumXCells() == 1 || sdf.GetNumYCells() == 1 || sdf.GetNumZCells() == 1)
    {
        std::cout << "SDF will be 2D - requires 8SSEDT" << std::endl;
        sdf_cell empty_cell;
        empty_cell.d1 = INFINITY;
        empty_cell.d2 = INFINITY;
        sdf_cell filled_cell;
        filled_cell.d1 = 0.0;
        filled_cell.d2 = 0.0;
        // Mark empty/filled cells
        DistanceField2D empty_cells(sdf.GetOriginTransform(), sdf.GetResolution(), sdf.GetXSize(), sdf.GetYSize(), sdf.GetZSize(), empty_cell);
        DistanceField2D filled_cells(sdf.GetOriginTransform(), sdf.GetResolution(), sdf.GetXSize(), sdf.GetYSize(), sdf.GetZSize(), filled_cell);
        for (int64_t x_index = 0; x_index < sdf.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < sdf.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < sdf.GetNumZCells(); z_index++)
                {
                    // Get the real world location
                    std::vector<double> location = sdf.GridIndexToLocation(x_index, y_index, z_index);
                    double x = location[0];
                    double y = location[1];
                    double z = location[2];
                    Eigen::Vector3d position(x, y, z);
                    // Lookup the location in the grid
                    CELL_STATE cell_state = QueryMultiresolutionGrid(position, query_mode).first;
                    if (cell_state == CELL_STATE::KNOWN_FILLED || cell_state == CELL_STATE::LIKELY_FILLED)
                    {
                        // Mark as filled
                        filled_cells.SetValue(x_index, y_index, z_index, filled_cell);
                        empty_cells.SetValue(x_index, y_index, z_index, empty_cell);
                    }
                    else if (cell_state == CELL_STATE::KNOWN_EMPTY || cell_state == CELL_STATE::LIKELY_EMPTY)
                    {
                        // Mark as free space
                        filled_cells.SetValue(x_index, y_index, z_index, empty_cell);
                        empty_cells.SetValue(x_index, y_index, z_index, filled_cell);
                    }
                    else
                    {
                        if (mark_unknown_as_filled)
                        {
                            // Mark as filled
                            filled_cells.SetValue(x_index, y_index, z_index, filled_cell);
                            empty_cells.SetValue(x_index, y_index, z_index, empty_cell);
                        }
                        else
                        {
                            // Mark as free space
                            filled_cells.SetValue(x_index, y_index, z_index, empty_cell);
                            empty_cells.SetValue(x_index, y_index, z_index, filled_cell);
                        }
                    }
                }
            }
        }
        // Process the fields
        Build2DDistanceField(filled_cells);
        Build2DDistanceField(empty_cells);
        // Generate the SDF
        for (int64_t x_index = 0; x_index < sdf.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < sdf.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < sdf.GetNumZCells(); z_index++)
                {
                    double filled_distance = sqrt(pow(filled_cells.GetImmutable(x_index, y_index, z_index).first.d1, 2) + pow(filled_cells.GetImmutable(x_index, y_index, z_index).first.d2, 2)) * sdf.GetResolution();
                    double empty_distance = sqrt(pow(empty_cells.GetImmutable(x_index, y_index, z_index).first.d1, 2) + pow(empty_cells.GetImmutable(x_index, y_index, z_index).first.d2, 2)) * sdf.GetResolution();
                    sdf.Set(x_index, y_index, z_index, (filled_distance - empty_distance));
                }
            }
        }
        return sdf;
    }
    else
    {
        std::cout << "SDF will be 3D - requires bucket cell" << std::endl;
        // Loop through the multiresolution grid and mark empty/filled cells
        std::vector<Eigen::Vector3i> filled;
        std::vector<Eigen::Vector3i> free;
        for (int64_t x_index = 0; x_index < sdf.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < sdf.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < sdf.GetNumZCells(); z_index++)
                {
                    // Get the real world location
                    std::vector<double> location = sdf.GridIndexToLocation(x_index, y_index, z_index);
                    double x = location[0];
                    double y = location[1];
                    double z = location[2];
                    Eigen::Vector3d position(x, y, z);
                    Eigen::Vector3i index(x_index, y_index, z_index);
                    // Lookup the location in the grid
                    CELL_STATE cell_state = QueryMultiresolutionGrid(position, query_mode).first;
                    if (cell_state == CELL_STATE::KNOWN_FILLED || cell_state == CELL_STATE::LIKELY_FILLED)
                    {
                        // Mark as filled
                        filled.push_back(index);
                    }
                    else if (cell_state == CELL_STATE::KNOWN_EMPTY || cell_state == CELL_STATE::LIKELY_EMPTY)
                    {
                        // Mark as free space
                        free.push_back(index);
                    }
                    else
                    {
                        if (mark_unknown_as_filled)
                        {
                            // Mark as filled
                            filled.push_back(index);
                        }
                        else
                        {
                            // Mark as free space
                            free.push_back(index);
                        }
                    }
                }
            }
        }
        // Make two distance fields (one for distance to filled voxels, one for distance to free voxels
        DistanceField3D filled_distance_field = Build3DDistanceField(filled, sdf.GetResolution());
        DistanceField3D free_distance_field = Build3DDistanceField(free, sdf.GetResolution());
        // Generate the SDF
        for (int64_t x_index = 0; x_index < sdf.GetNumXCells(); x_index++)
        {
            for (int64_t y_index = 0; y_index < sdf.GetNumYCells(); y_index++)
            {
                for (int64_t z_index = 0; z_index < sdf.GetNumZCells(); z_index++)
                {
                    double distance1 = sqrt(filled_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * sdf.GetResolution();
                    double distance2 = sqrt(free_distance_field.GetImmutable(x_index, y_index, z_index).first.distance_square) * sdf.GetResolution();
                    sdf.Set(x_index, y_index, z_index, (distance1 - distance2));
                }
            }
        }
        return sdf;
    }
}

visualization_msgs::Marker MultiresolutionVoxelGridManager::ExportForDisplay(QUERY_MODE query_mode, float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "multiresolution_voxelgrid_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = grid_.GetHighResolution();
    display_rep.scale.y = grid_.GetHighResolution();
    display_rep.scale.z = grid_.GetHighResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < grid_.GetHighResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetHighResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetHighResolutionNumZCells(); z_index++)
            {
                // Get the real-world location of the cell
                std::vector<double> location = grid_.HighResolutionGridIndexToLocation(x_index, y_index, z_index);
                if (location.size() == 3)
                {
                    Eigen::Vector3d cell_position(location[0], location[1], location[2]);
                    // Lookup the location in the grid
                    CELL_STATE cell_value = QueryMultiresolutionGrid(cell_position, query_mode).first;
                    if (cell_value == CELL_STATE::KNOWN_FILLED)
                    {
                        geometry_msgs::Point new_point;
                        new_point.x = location[0];
                        new_point.y = location[1];
                        new_point.z = location[2];
                        display_rep.points.push_back(new_point);
                        std_msgs::ColorRGBA new_color;
                        new_color.a = alpha;
                        new_color.b = 0.0;
                        new_color.g = 0.0;
                        new_color.r = 1.0;
                        display_rep.colors.push_back(new_color);
                    }
                    else if (cell_value == CELL_STATE::LIKELY_FILLED)
                    {
                        geometry_msgs::Point new_point;
                        new_point.x = location[0];
                        new_point.y = location[1];
                        new_point.z = location[2];
                        display_rep.points.push_back(new_point);
                        std_msgs::ColorRGBA new_color;
                        new_color.a = alpha;
                        new_color.b = 0.0;
                        new_color.g = 0.0;
                        new_color.r = 0.5;
                        display_rep.colors.push_back(new_color);
                    }
                    else if (cell_value == CELL_STATE::KNOWN_EMPTY)
                    {
                        geometry_msgs::Point new_point;
                        new_point.x = location[0];
                        new_point.y = location[1];
                        new_point.z = location[2];
                        display_rep.points.push_back(new_point);
                        std_msgs::ColorRGBA new_color;
                        new_color.a = alpha;
                        new_color.b = 1.0;
                        new_color.g = 0.0;
                        new_color.r = 0.0;
                        display_rep.colors.push_back(new_color);
                    }
                    else if (cell_value == CELL_STATE::LIKELY_EMPTY)
                    {
                        geometry_msgs::Point new_point;
                        new_point.x = location[0];
                        new_point.y = location[1];
                        new_point.z = location[2];
                        display_rep.points.push_back(new_point);
                        std_msgs::ColorRGBA new_color;
                        new_color.a = alpha;
                        new_color.b = 0.5;
                        new_color.g = 0.0;
                        new_color.r = 0.0;
                        display_rep.colors.push_back(new_color);
                    }
                    else
                    {
                        geometry_msgs::Point new_point;
                        new_point.x = location[0];
                        new_point.y = location[1];
                        new_point.z = location[2];
                        display_rep.points.push_back(new_point);
                        std_msgs::ColorRGBA new_color;
                        new_color.a = alpha;
                        new_color.b = 0.5;
                        new_color.g = 0.5;
                        new_color.r = 0.5;
                        display_rep.colors.push_back(new_color);
                    }
                }
            }
        }
    }
    return display_rep;
}

visualization_msgs::Marker MultiresolutionVoxelGridManager::ExportHighResolutionForDisplay(float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "multiresolution_voxelgrid_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = grid_.GetHighResolution();
    display_rep.scale.y = grid_.GetHighResolution();
    display_rep.scale.z = grid_.GetHighResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < grid_.GetHighResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetHighResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetHighResolutionNumZCells(); z_index++)
            {
                // Get the value of the cell
                CELL_STATE cell_value = grid_.CheckHighResolutionGrid(x_index, y_index, z_index).first;
                // Get the real-world location of the cell
                std::vector<double> location = grid_.HighResolutionGridIndexToLocation(x_index, y_index, z_index);
                if (cell_value == CELL_STATE::KNOWN_FILLED)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 1.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::LIKELY_FILLED)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 0.5;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::KNOWN_EMPTY)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 1.0;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::LIKELY_EMPTY)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.5;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::UNKNOWN)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.5;
                    new_color.g = 0.5;
                    new_color.r = 0.5;
                    display_rep.colors.push_back(new_color);
                }
                else
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
            }
        }
    }
    return display_rep;
}

visualization_msgs::Marker MultiresolutionVoxelGridManager::ExportLowResolutionForDisplay(float alpha)
{
    // Assemble a visualization_markers::Marker representation of the SDF to display in RViz
    visualization_msgs::Marker display_rep;
    // Populate the header
    display_rep.header.frame_id = frame_;
    // Populate the options
    display_rep.ns = "multiresolution_voxelgrid_display";
    display_rep.id = 1;
    display_rep.type = visualization_msgs::Marker::CUBE_LIST;
    display_rep.action = visualization_msgs::Marker::ADD;
    display_rep.lifetime = ros::Duration(0.0);
    display_rep.frame_locked = false;
    display_rep.scale.x = grid_.GetLowResolution();
    display_rep.scale.y = grid_.GetLowResolution();
    display_rep.scale.z = grid_.GetLowResolution();
    // Add all the cells of the SDF to the message
    for (int64_t x_index = 0; x_index < grid_.GetLowResolutionNumXCells(); x_index++)
    {
        for (int64_t y_index = 0; y_index < grid_.GetLowResolutionNumYCells(); y_index++)
        {
            for (int64_t z_index = 0; z_index < grid_.GetLowResolutionNumZCells(); z_index++)
            {
                // Get the value of the cell
                CELL_STATE cell_value = grid_.CheckLowResolutionGrid(x_index, y_index, z_index).first;
                // Get the real-world location of the cell
                std::vector<double> location = grid_.LowResolutionGridIndexToLocation(x_index, y_index, z_index);
                if (cell_value == CELL_STATE::KNOWN_FILLED)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 1.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::LIKELY_FILLED)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 0.5;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::KNOWN_EMPTY)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 1.0;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::LIKELY_EMPTY)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.5;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
                else if (cell_value == CELL_STATE::UNKNOWN)
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.5;
                    new_color.g = 0.5;
                    new_color.r = 0.5;
                    display_rep.colors.push_back(new_color);
                }
                else
                {
                    geometry_msgs::Point new_point;
                    new_point.x = location[0];
                    new_point.y = location[1];
                    new_point.z = location[2];
                    display_rep.points.push_back(new_point);
                    std_msgs::ColorRGBA new_color;
                    new_color.a = alpha;
                    new_color.b = 0.0;
                    new_color.g = 0.0;
                    new_color.r = 0.0;
                    display_rep.colors.push_back(new_color);
                }
            }
        }
    }
    return display_rep;
}

sdf_tools::SignedDistanceField MultiresolutionVoxelGridManager::BuildLowResolutionSDF(QUERY_MODE query_mode, float OOB_value, bool mark_unknown_as_filled)
{
    return BuildSDF(query_mode, grid_.GetLowResolution(), OOB_value, mark_unknown_as_filled);
}

sdf_tools::SignedDistanceField MultiresolutionVoxelGridManager::BuildHighResolutionSDF(QUERY_MODE query_mode, float OOB_value, bool mark_unknown_as_filled)
{
    return BuildSDF(query_mode, grid_.GetHighResolution(), OOB_value, mark_unknown_as_filled);
}
