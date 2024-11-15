#include "transform_data.hpp"

void transform_data(
    std::shared_ptr<open3d::geometry::PointCloud>& source, 
    data_packet* packet_ptr,
    size_t num_points_offset
) {
    int points_num = source->points_.size();
    int offset = num_points_offset * sizeof(double) * 3;
    for (int i = 0; i < points_num; i++) {
        packet_ptr->float_array_ptr[int(offset) + i * 3] = static_cast<double>(source->points_[i].x());
        packet_ptr->float_array_ptr[int(offset) + i * 3 + 1] = static_cast<double>(source->points_[i].y());
        packet_ptr->float_array_ptr[int(offset) + i * 3 + 2] = static_cast<double>(source->points_[i].z());
    }
    return;
}