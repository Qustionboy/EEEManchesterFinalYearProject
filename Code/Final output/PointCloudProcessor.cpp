#include "PointCloudProcessor.h"

void PointCloudProcessor::processPointCloud(int numSlices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);


    std::cout << "Attempting to read PCD file from: " << inputFile << std::endl;

    // Read the point cloud
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(inputFile, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return;
    }

    // Downsample the point cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f); // Adjusted voxel size
    sor.filter(*downsampledCloud);

    // Find min and max Z values
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();
    for (const auto& point : downsampledCloud->points) {
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }

    // Determine the slice height
    float sliceHeight = (maxZ - minZ) / numSlices;

    // Slice the point cloud and save slices as PGM
    for (int i = 0; i < numSlices; ++i) {
        float zLower = minZ + i * sliceHeight;
        float zUpper = minZ + (i + 1) * sliceHeight;
        std::string filename = outputDir + "/slice_" + std::to_string(i) + ".pgm";

        // Initialize an empty grid for a larger PGM
        std::vector<std::vector<int>> grid(500, std::vector<int>(500, 0));

        // Fill the grid
        for (const auto& point : downsampledCloud->points) {
            if (point.z >= zLower && point.z < zUpper) {
                // Adjust scale and offset for a larger image size
                int x = static_cast<int>((point.x - cloud->sensor_origin_.x()) * 100) + 250;
                int y = static_cast<int>((point.y - cloud->sensor_origin_.y()) * 100) + 250;
                if (x >= 0 && x < 500 && y >= 0 && y < 500) {
                    grid[y][x] += 1; // Increase density
                }
            }
        }

        // Save the slice as a PGM
        std::ofstream outFile(filename);
        outFile << "P2\n500 500\n255\n";
        for (const auto& row : grid) {
            for (const auto& cell : row) {
                int intensity = std::min(cell * 5, 255); // Scale intensity for visibility
                outFile << intensity << " ";
            }
            outFile << "\n";
        }
        outFile.close();
    }
}