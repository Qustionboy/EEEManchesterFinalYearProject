#define _USE_MATH_DEFINES
#include "point_cloud_creation.h"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

PointCloudCreation::PointCloudCreation() {
    lsrule.generateFractal();
}

void PointCloudCreation::drawCylinder(double r, double h) {
    temp_stem.clear(); // Clear previous data

    for (int i = 0; i < 360; i += lsrule.cylinder_step) {
        double temp = i * M_PI / 180;
        double temp1 = (i + lsrule.cylinder_step) * M_PI / 180;

        // Radius shrinkage factor for the top circle
        double top_shrink = grape.trunk.radius_shrink;

        // Bottom circle
        temp_stem.emplace_back(r * std::cos(temp), 0.0, r * std::sin(temp));
        //temp_stem.emplace_back(top_shrink * r * std::cos(temp1), h/2, top_shrink * r * std::sin(temp1));

    }
}

void PointCloudCreation::drawChannel(Node A, Node B, double r) {
    // Calculate the direction vector of the cylinder
    Eigen::Vector3f pointA(A.x, A.y, A.z);
    Eigen::Vector3f pointB(B.x, B.y, B.z);
    Eigen::Vector3f direction = (pointB - pointA).normalized();

    // Calculate the height of the Channel
    double h = (pointA - pointB).norm();

    //Draw the Cylinder and store it into temp_stem
    drawCylinder(r, h);

    //Calculate the rotation matrix
    Eigen::Vector3f yAxis(0, 1, 0); 
    Eigen::Vector3f rotationAxis = yAxis.cross(direction).normalized();
    float angle = std::acos(yAxis.dot(direction));
    Eigen::Affine3f rotationMatrix = Eigen::Affine3f(Eigen::AngleAxisf(angle, rotationAxis));
    
    //Calculate the translation matrix
    Eigen::Translation3f translationMatrix(A.x, A.y, A.z);

    // Combine the rotation and transformation
    Eigen::Affine3f transform = translationMatrix * rotationMatrix;

    for (auto& vertex : temp_stem) {
        Eigen::Vector3f point(vertex.x, vertex.y, vertex.z);
        Eigen::Vector3f transformedPoint = transform * point;
        vertex.x = transformedPoint.x();
        vertex.y = transformedPoint.y();
        vertex.z = transformedPoint.z();
    }

}

void PointCloudCreation::drawBerry(Node A, Node dir, double r) {
    Eigen::Vector3f pointA(A.x, A.y, A.z);
    Eigen::Vector3f pointB(dir.x, dir.y, dir.z);
    
    Eigen::Vector3f direction = pointB.normalized();
    // Draw the berry
    temp_berry = grape.berry.createTaperedSphere(r, lsrule.longs, lsrule.lats);

    //Calculate the rotation matrix
    Eigen::Vector3f yAxis(0, 1, 0);
    Eigen::Vector3f rotationAxis = yAxis.cross(direction).normalized();
    float angle = std::acos(yAxis.dot(direction));
    Eigen::Affine3f rotationMatrix = Eigen::Affine3f(Eigen::AngleAxisf(angle, rotationAxis));

    //Calculate the translation matrix
    Eigen::Translation3f translationMatrix(A.x, A.y, A.z);

    // Combine the rotation and transformation
    Eigen::Affine3f transform = translationMatrix * rotationMatrix;

    for (auto& vertex : temp_berry) {
        Eigen::Vector3f point(vertex.x, vertex.y, vertex.z);
        Eigen::Vector3f transformedPoint = transform * point;
        vertex.x = transformedPoint.x();
        vertex.y = transformedPoint.y();
        vertex.z = transformedPoint.z();
    }

}


void PointCloudCreation::drawStem(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (unsigned int i = 0; i < lsrule.trunks.size(); i++) {
        drawChannel(lsrule.trunks[i].pos1, lsrule.trunks[i].pos2, lsrule.trunks[i].radius);

        for (const auto& node : temp_stem) {
            pcl::PointXYZRGB point;
            point.x = node.x;
            point.y = node.y;
            point.z = node.z;
            point.r = 119;
            point.g = 238;
            point.b = 0;
            stem_cloud->push_back(point);
        }
    }

    pcl::io::savePCDFileBinary("stem.pcd", *stem_cloud);
    std::cout << "Saved stem with " << stem_cloud->size() << " points to stem.pcd." << std::endl;
}

void PointCloudCreation::SpaceCalculaterI() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("grape.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);
    //*cloud_filtered += *cloud;
    pcl::io::savePCDFileBinary("filtered.pcd", *cloud_filtered);
    // Find the maximum point in the cloud
    minX = std::numeric_limits<float>::max();
    maxX = -std::numeric_limits<float>::max();
    minY = std::numeric_limits<float>::max();
    maxY = -std::numeric_limits<float>::max();
    minZ = std::numeric_limits<float>::max();
    maxZ = -std::numeric_limits<float>::max();
    for (const auto& point : cloud->points) {
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }
    float max_height = (maxZ - minZ); 
    slice_thickness = max_height / num_slices;
    x_scale = image_width / (maxX - minX);
    y_scale = image_height / (maxY - minY);
    z_scale = image_height / (maxZ - minZ);

}

void PointCloudCreation::GenerateSlicePGMY() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("filtered.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
    }
    for (int i = 0; i < num_slices; ++i) {
        float y_min = minY + i * slice_thickness;
        float y_max = minY + (i + 1) * slice_thickness;

        std::vector<std::vector<int>> grid(image_height, std::vector<int>(image_width, 0));

        for (const auto& point : cloud->points) {
            if (point.y >= y_min && point.y < y_max) {
                int x = static_cast<int>((point.x - minX) * x_scale);
                int z = static_cast<int>((point.z - minZ) * z_scale);
                if (x >= 0 && x < image_width && z >= 0 && z < image_height) {
                    
                    grid[z][x] = std::min(static_cast<int>((point.z + 20) * intensity_scale), 255);
                }
            }
        }

        // Save PGM 
        std::string filename = "slice_y_" + std::to_string(i) + ".pgm";
        std::ofstream ofs(filename, std::ios::out);
        ofs << "P2\n" << image_width << " " << image_height << "\n255\n";
        for (const auto& depth_row : grid) {
            for (const auto& val : depth_row) {
                ofs << val << " ";
            }
            ofs << "\n";
        }
        ofs.close();
    }
}
void PointCloudCreation::GenerateSlicePGMZ() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("filtered.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
    }
    for (int i = 0; i < num_slices; ++i) {
        float z_min = minZ + i * slice_thickness;
        float z_max = minZ + (i + 1) * slice_thickness;

        std::vector<std::vector<int>> grid(image_height, std::vector<int>(image_width, 0));

        for (const auto& point : cloud->points) {
            if (point.z >= z_min && point.z < z_max) {
                int x = static_cast<int>((point.x - minX) * x_scale);
                int y = static_cast<int>((point.y - minY) * y_scale);
                if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
                
                    grid[y][x] = std::min(static_cast<int>((point.z + 20) * intensity_scale), 255);
                }
            }
        }

        // 保存切片为PGM文件
        std::string filename = "slice_" + std::to_string(i) + ".pgm";
        std::ofstream ofs(filename, std::ios::out);
        ofs << "P2\n" << image_width << " " << image_height << "\n255\n";
        for (const auto& row : grid) {
            for (const auto& val : row) {
                ofs << val << " ";
            }
            ofs << "\n";
        }
        ofs.close();
    }

}
void PointCloudCreation::GenerateFullModelPGM(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("filtered.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input.pcd \n");
    }
    std::vector<std::vector<int>> grid(image_height, std::vector<int>(image_width, 0));

    for (const auto& point : cloud->points) {
        int x = static_cast<int>((point.x - minX) * x_scale);
        int y = static_cast<int>((point.y - minY) * y_scale);
        if (x >= 0 && x < image_width && y >= 0 && y < image_height) {
            grid[y][x] = std::min(static_cast<int>((point.z+20) * intensity_scale), 255);
        }
    }

    std::ofstream ofs(filename, std::ios::out);
    ofs << "P2\n" << image_width << " " << image_height << "\n255\n";
    for (const auto& row : grid) {
        for (const auto& val : row) {
            ofs << val << " ";
        }
        ofs << "\n";
    }
    ofs.close();
    std::cout << "Saved full model to " << filename << std::endl;
}


void PointCloudCreation::ResultGenerator_KD() {
    try {
       
        //initate the point cloud and kd tree
        pcl::PointCloud<pcl::PointXYZ>::Ptr centers_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_stem;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr stem_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr stem_centre(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr grape_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << "Starting generation with " << lsrule.trunks.size() << " trunks." << std::endl;
        
        //drawStem();
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("stem.pcd", *stem_cloud) == -1) {
            PCL_ERROR("Couldn't read file\n");
        }

        *grape_cloud += *stem_cloud;

        for (unsigned int i = 0; i < lsrule.trunks.size(); i++) {
            pcl::PointXYZ StemPos(lsrule.trunks[i].pos1.x, lsrule.trunks[i].pos1.y, lsrule.trunks[i].pos1.z);
            stem_centre->push_back(StemPos);
        }
        
        std::cout << "Starting generation with " << lsrule.berries.size() << " berries." << std::endl;
        //initate the kd tree as kd tree cannot be empty
        pcl::PointXYZ initialPoint(0.0f, 0.0f, 0.0f);
        centers_cloud->push_back(initialPoint); 
        kdtree.setInputCloud(centers_cloud); 
        kdtree_stem.setInputCloud(stem_centre);
        int number_berry = 0;
        for (int i = 0; i < lsrule.berries.size(); i++) {
            double radius = lsrule.berries[i].radius;  
            bool hasStemOverlap = false;

            temp_berry.clear();
            drawBerry(lsrule.berries[i].centres, lsrule.berries[i].direction, radius);

            pcl::PointXYZ searchPoint(lsrule.berries[i].centres.x, lsrule.berries[i].centres.y, lsrule.berries[i].centres.z);

 
            std::vector<int> pointIdxRadiusSearchStem;
            std::vector<float> pointRadiusSquaredDistanceStem;
            if (kdtree_stem.radiusSearch(searchPoint, radius, pointIdxRadiusSearchStem, pointRadiusSquaredDistanceStem) > 0) {
                hasStemOverlap = true;  
            }

    
            if (!hasStemOverlap) {
                bool hasOverlap = false;  
                std::vector<int> pointIdxRadiusSearch;
                std::vector<float> pointRadiusSquaredDistance;

                if (kdtree.radiusSearch(searchPoint, 2 * radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                    radius *= 0.6;
                    lsrule.NewPOS(lsrule.berries[i].radius, radius, i);
                    temp_berry.clear();
                    drawBerry(lsrule.berries[i].centres, lsrule.berries[i].direction, radius);  

                    if (kdtree.radiusSearch(searchPoint, 2 * radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                        hasOverlap = true;
                    }
                }

                
                if (!hasOverlap) {
                    centers_cloud->push_back(searchPoint);
                    kdtree.setInputCloud(centers_cloud);

                    number_berry++;
                    for (const auto& node : temp_berry) {
                        pcl::PointXYZRGB point;
                        point.x = node.x;
                        point.y = node.y;
                        point.z = node.z;
                        point.r = 128;
                        point.g = 0;
                        point.b = 128;
                        grape_cloud->push_back(point);
                    }
                }
            }
        }


        pcl::io::savePCDFileBinary("grape.pcd", *grape_cloud);
        std::cout << "Number of berries that actually generates: " << number_berry << std::endl;
        std::cout << "Saved grape model with " << grape_cloud->size() << " points to grape.pcd." << std::endl;
    }

    catch (const std::exception& e) {
        std::cerr << "An exception occurred: " << e.what() << std::endl;
    }
}

