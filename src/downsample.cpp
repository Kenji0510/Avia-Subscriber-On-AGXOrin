#include "downsample.hpp"


void downsample(std::shared_ptr<open3d::geometry::PointCloud>& source, std::shared_ptr<open3d::geometry::PointCloud>& downsampled_source) {
    double voxel_size = 0.1;
    int nb_neighbors = 10;
    double std_ratio = 1;     // 0.1 is not adopted.
    
    auto source_tensor = open3d::t::geometry::PointCloud::FromLegacy(*source, open3d::core::Dtype::Float32, open3d::core::Device("CUDA:0"));

    auto start_to_voxel = std::chrono::high_resolution_clock::now();

    auto voxel_grid = source_tensor.VoxelDownSample(voxel_size);

    auto finish_to_voxel = std::chrono::high_resolution_clock::now();

    // For remove noise
    auto [remove_noise_pcd, ind] = voxel_grid.RemoveStatisticalOutliers(nb_neighbors, std_ratio);
    open3d::t::geometry::PointCloud remove_noise_pcd_legacy = std::move(remove_noise_pcd);

    auto finish_to_remove_noise = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> elapsed_voxel = finish_to_voxel - start_to_voxel;
    std::cout << "\e[32m" << "Voxel Process Time: " << elapsed_voxel.count() << " ms" << "\e[m" << std::endl;
    std::chrono::duration<double, std::milli> elapsed_remove_noise = finish_to_remove_noise - start_to_voxel;
    std::cout << "\e[32m" << "Remove noise Process Time: " << elapsed_remove_noise.count() << " ms" << "\e[m" << std::endl;

    // Convert to legacy.(Copy the pcd from gpu to cpu)
    downsampled_source = std::make_shared<open3d::geometry::PointCloud>(remove_noise_pcd_legacy.ToLegacy());

}