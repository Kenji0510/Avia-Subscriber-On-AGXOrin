#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <open3d/Open3D.h>

void downsample(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& downsampled_source);