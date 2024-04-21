#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <limits>

class PointCloudProcessor {
public:
    PointCloudProcessor(const std::string& inputFile, const std::string& outputDir)
        : inputFile(inputFile), outputDir(outputDir) {}

    void processPointCloud(int numSlices);

private:
    std::string inputFile;
    std::string outputDir;
};

