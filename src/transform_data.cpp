#include "transform_data.hpp"

void transform_data(std::shared_ptr<open3d::geometry::PointCloud>& source, data_packet* packet_ptr) {
    int points_num = source->points_.size();
    for (int i = 0; i < points_num; i++) {
        packet_ptr->float_array_ptr[i * 3] = static_cast<double>(source->points_[i].x());
        packet_ptr->float_array_ptr[i * 3 + 1] = static_cast<double>(source->points_[i].y());
        packet_ptr->float_array_ptr[i * 3 + 2] = static_cast<double>(source->points_[i].z());
    }
    return;
}