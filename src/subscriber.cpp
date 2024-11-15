#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <open3d/Open3D.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>

#include "downsample.hpp"
#include "send_data.h"
#include "transform_data.hpp"
#include "subscriber.hpp"


data_packet *data_packet_ptr = NULL;
Thread_Control_Flag *thread_control_flag = NULL;
std::condition_variable push_cv;
bool can_push_data_laser_map = false;
std::condition_variable send_cv;
bool can_send_pcd = false;
const int MAX_DATA_SIZE = 100;
size_t count = 0;


int imu_callback_count = 0;
ros::Time last_time;

size_t num_points_laser_map = 0;


std::atomic<bool> stop_send_flag(true); // To wait send process untill 15s from initial running 
ros::Time last_callback_time;   // To gets point cloud data from /Laser_map topic(FAST-LIO) at 3Hz

std::string createTimestampedFilename(std::string path) {
    //std::string path = "/home/kenji/pcd";
    // Get Now Time
    auto now = std::chrono::system_clock::now();
    auto inTimeT = std::chrono::system_clock::to_time_t(now);

    // Transfer Times to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&inTimeT), "%Y%m%d_%H%M%S");

    // Create FileName
    //std::string fileName = path + "/saved_cloud_pub_freq_" + std::to_string(2) + "_" + ss.str() + ".pcd";
    //std::string file_name = path + "/pub_freq_" + std::to_string(2) + "_" + "voxel_size_01_nb_neighbors_30_std_ratio_01_" + ss.str() + ".pcd";
    std::string file_name = path + "/pub_freq_" + std::to_string(2) + "_" + ss.str() + ".pcd";
    return file_name;
}

void callback_for_laser_map(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (stop_send_flag == true) {
        return;
    }

    ros::Time current_time = ros::Time::now();
    if((current_time - last_callback_time).toSec() < (1.0 / 2.0)) { // Default is 3Hz
        return;
    } else {
        last_callback_time = current_time;
    }

    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto source_downsampled = std::make_shared<open3d::geometry::PointCloud>();
    //Eigen::Vector3d delta_angle = Eigen::Vector3d::Zero();

    // if (data_packet_ptr == NULL) {
    //     data_packet_ptr = (data_packet*)malloc(sizeof(data_packet));
    //     data_packet_ptr->float_array_ptr = (double*)malloc(sizeof(double) * NUM_FLOATS * 3);
    // }

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Transform the point cloud data from PCL PointCloud to Open3D PointCloud
    for (const auto& point : pcl_cloud->points) {
        source->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    printf("\e[36m<--- /Laser_map process --->\e[0m\n");
    printf("\e[33m<--- Start of session --->\e[0m\n");

    {   
        std::lock_guard<std::mutex> lock(thread_control_flag->access_gpu_mutex);
        // Downsample the point cloud and remove noise
        downsample(source, source_downsampled);
    }
    //data_packet_ptr->after_voxel_time = time(NULL);

    //data_packet_ptr->unix_time = time(NULL);
    data_packet_ptr->num_points_of_laser_map = source_downsampled->points_.size();
    // data_packet_ptr->angle_velocity_x = delta_angle(0);
    // data_packet_ptr->angle_velocity_y = delta_angle(1);
    // data_packet_ptr->angle_velocity_z = delta_angle(2);
    // data_packet_ptr->roll = roll;
    // data_packet_ptr->pitch = pitch;
    // data_packet_ptr->yaw = yaw;
    {
        // Transform the point cloud data from Open3D PointCloud to data_packet
        std::lock_guard<std::mutex> lock(thread_control_flag->float_array_ptr_mutex);
        transform_data(source_downsampled, data_packet_ptr, 0);
        num_points_laser_map = data_packet_ptr->num_points_of_laser_map;
        can_push_data_laser_map = true;
    }
    push_cv.notify_one();

    // Send the point cloud data to the server
    //int run_result = send_data("180.145.242.113", "1234", data_packet_ptr);

    // Timesstamped filename
    //std::string file_path = "/home/kenji/pcd";
    std::string file_path = "/home/kenji/ws_livox/src/lidar_subscriber/data/fast-lio";
    std::string file_name = createTimestampedFilename(file_path);

    count += 1;

    printf("\e[36m<--- End of session --->\e[0m\n");
    printf("\e[36m<--- /Laser_map process --->\e[0m\n");

    // Save to a PCD file
    //pcl::io::savePCDFileASCII(fileName, *pcl_cloud);

    // Save to a PCD file
    // bool result = open3d::io::WritePointCloud(file_name, *source_downsampled);
    // if (result) {
    //     std::cout << "\e[32m" << "Successfully saved the point cloud." << "\e[m" << std::endl;
    // } else {
    //     std::cerr << "\e[31m" << "Failed to save the point cloud." << "\e[m" << std::endl;
    // }

    //ROS_INFO("Saved %d data points to %s", cloud->size(), fileName.c_str());
}

void callback_for_cloud_registered(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    if (stop_send_flag == true) {
        return;
    }

    ros::Time current_time = ros::Time::now();
    if((current_time - last_callback_time).toSec() < (1.0 / 2.0)) { // Default is 3Hz
        return;
    } else {
        last_callback_time = current_time;
    }

    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto source_downsampled = std::make_shared<open3d::geometry::PointCloud>();
    //Eigen::Vector3d delta_angle = Eigen::Vector3d::Zero();

    // if (data_packet_ptr == NULL) {
    //     data_packet_ptr = (data_packet*)malloc(sizeof(data_packet));
    //     data_packet_ptr->float_array_ptr = (double*)malloc(sizeof(double) * NUM_FLOATS * 3);
    // }

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    // Transform the point cloud data from PCL PointCloud to Open3D PointCloud
    for (const auto& point : pcl_cloud->points) {
        source->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    printf("\e[36m<--- /cloud_registered process --->\e[0m\n");
    printf("\e[33m<--- Start of session --->\e[0m\n");

    {   
        std::lock_guard<std::mutex> lock(thread_control_flag->access_gpu_mutex);
        // Downsample the point cloud and remove noise
        downsample(source, source_downsampled);
    }
    //data_packet_ptr->after_voxel_time = time(NULL);

    //data_packet_ptr->unix_time = time(NULL);
    data_packet_ptr->num_points_of_cloud_registered = source_downsampled->points_.size();
    // data_packet_ptr->angle_velocity_x = delta_angle(0);
    // data_packet_ptr->angle_velocity_y = delta_angle(1);
    // data_packet_ptr->angle_velocity_z = delta_angle(2);
    // data_packet_ptr->roll = roll;
    // data_packet_ptr->pitch = pitch;
    // data_packet_ptr->yaw = yaw;

    {
        // Transform the point cloud data from Open3D PointCloud to data_packet
        std::unique_lock<std::mutex> lock(thread_control_flag->float_array_ptr_mutex);
        push_cv.wait(lock, [] { return can_push_data_laser_map; });
        transform_data(source_downsampled, data_packet_ptr, num_points_laser_map);
        can_push_data_laser_map = false;
        can_send_pcd = true;
    }

    // Send the point cloud data to the server
    //int run_result = send_data("180.145.242.113", "1234", data_packet_ptr);

    // Timesstamped filename
    //std::string file_path = "/home/kenji/pcd";
    std::string file_path = "/home/kenji/ws_livox/src/lidar_subscriber/data/fast-lio";
    std::string file_name = createTimestampedFilename(file_path);

    count += 1;

    printf("\e[36m<--- End of session --->\e[0m\n");
    printf("\e[36m<--- /cloud_registered process --->\e[0m\n");

    // Save to a PCD file
    //pcl::io::savePCDFileASCII(fileName, *pcl_cloud);

    // Save to a PCD file
    // bool result = open3d::io::WritePointCloud(file_name, *source_downsampled);
    // if (result) {
    //     std::cout << "\e[32m" << "Successfully saved the point cloud." << "\e[m" << std::endl;
    // } else {
    //     std::cerr << "\e[31m" << "Failed to save the point cloud." << "\e[m" << std::endl;
    // }

    //ROS_INFO("Saved %d data points to %s", cloud->size(), fileName.c_str());
}

void send_pcd_process() {
    // while (true) {
    //     if (stop_send_flag == true) {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         continue;
    //     }

    //     {
    //         // Transform the point cloud data from Open3D PointCloud to data_packet
    //         std::unique_lock<std::mutex> lock(thread_control_flag->float_array_ptr_mutex);
    //         send_cv.wait(lock, [] { return can_send_pcd; });
    //         int run_result = send_data("180.145.242.113", "1234", data_packet_ptr);
    //         can_send_pcd = false;
    //     }
    //     printf("\e[36m<--- send_pcd process --->\e[0m\n");
    // }
}


void timer_callback(const ros::TimerEvent&) {
    ROS_INFO("IMU callback was called %d times in the last second.", imu_callback_count);
    imu_callback_count = 0;  // カウントをリセット
}

void stop_send_process() {
    auto start_time = std::chrono::steady_clock::now();
    while (stop_send_flag) {
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

        std::cout << "\e[34m" << "Collecting entire point cloud data from /Laser_map topic(FAST-LIO)... " << seconds << " seconds have passed." << "\e[0m" << std::endl;

        if (seconds >= 1) {
            std::cout << "15 seconds have passed. Stopping thread..." << std::endl;
            stop_send_flag = false;
            break;
        }

        boost::this_thread::sleep_for(boost::chrono::seconds(1));
    }
}

int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "livox_point_cloud_listener");
    ros::NodeHandle nh;

    if (data_packet_ptr == NULL) {
        data_packet_ptr = (data_packet*)malloc(sizeof(data_packet));
        data_packet_ptr->float_array_ptr = (double*)malloc(sizeof(double) * NUM_FLOATS * 3);
        thread_control_flag = (Thread_Control_Flag*)malloc(sizeof(Thread_Control_Flag));
    }

    // Create a ROS subscriber
    //ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("livox/lidar", 1, callback);

    // Create a ROS subscriber for get entire point cloud data from /Laser_map topic(FAST-LIO)
    //ros::Subscriber sub1 = nh.subscribe<sensor_msgs::PointCloud2>("/Laser_map", 1, callback_for_laser_map);
    ros::Subscriber sub2 = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 1, callback_for_cloud_registered);

    boost::asio::io_service io_service;
    boost::asio::signal_set signals(io_service, SIGINT);

    signals.async_wait(
        [](const boost::system::error_code&, int) {
            std::cout << "\nSIGINT received. Stopping thread..." << std::endl;
            stop_send_flag = false;
            ros::shutdown();
        }
    );

    boost::thread io_service_thread([&io_service]() { io_service.run(); });
    boost::thread worker(stop_send_process);
    boost::thread send_pcd_worker(send_pcd_process);

    // 初期時間の設定
    last_time = ros::Time::now();
    last_callback_time = ros::Time::now();

    // Spin to continuously get data from callback
    // ros::AsyncSpinner spinner(2);
    // spinner.start();

    ros::spin();

    ros::waitForShutdown();

    io_service.stop();
    io_service_thread.join();
    worker.join();
    send_pcd_worker.join();

    if (data_packet_ptr != NULL) {
        free(data_packet_ptr->float_array_ptr);
        free(data_packet_ptr);
        free(thread_control_flag);
        std::cout << "\e[33m" << "Freed allocated memory." << "\e[m" << std::endl;
        //save_angular_velocity_data();
    }

    return 0;
}