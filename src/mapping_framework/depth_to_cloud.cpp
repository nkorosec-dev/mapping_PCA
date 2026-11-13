#include "mapping.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <iostream>
#include <vector>
#include <cstdint>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Missing depth image path\n";
        return 1;
    }

    const std::string imagePath = argv[1];

    const double fx = 450.0;
    const double fy = 440.0;
    const double cx = 424.0;
    const double cy = 240.0;

    const double depthScale = 0.001;

    cv::Mat depth = cv::imread(imagePath, cv::IMREAD_UNCHANGED);

    if (depth.empty() || depth.type() != CV_16UC1)
    {
        std::cerr << "Invalid 16-bit depth image\n";
        return 1;
    }

    std::vector<Eigen::Vector3d> cloud;
    cloud.reserve(depth.rows * depth.cols);

    for (int v = 0; v < depth.rows; ++v)
    {
        for (int u = 0; u < depth.cols; ++u)
        {
            uint16_t raw = depth.at<uint16_t>(v, u);

            if (raw == 0)
                continue;

            double Z = raw * depthScale;

            if (Z < 0.15 || Z > 2.0)
                continue;

            double X = (u - cx) * Z / fx;
            double Y = (v - cy) * Z / fy;

            cloud.emplace_back(X, Y, Z);
        }
    }

    SaveCSV(cloud, "real_cloud.csv");

    std::cout << "Resolution: "
              << depth.cols << " x " << depth.rows << "\n";

    std::cout << "Valid 3D points: "
              << cloud.size() << "\n";

    return 0;
}