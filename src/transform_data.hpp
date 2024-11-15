#include <open3d/Open3D.h>

#include "send_data.h"
#include "subscriber.hpp"


void transform_data(
    std::shared_ptr<open3d::geometry::PointCloud>& source, 
    data_packet* packet_ptr,
    size_t num_points_offset
);