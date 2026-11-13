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

            if (Z < 0.25 || Z > 2.0)
                continue;

            double X = (u - cx) * Z / fx;
            double Y = (v - cy) * Z / fy;

            cloud.emplace_back(X, Y, Z);
        }
    }

    SaveCSV(cloud, "real_cloud.csv");

    std::mt19937 generator(42);

    // Global PCA
    PlaneModel globalPCA = FitPlanePCA(cloud);

    // RANSAC + PCA refinement
    PlaneModel plane = RansacPlane(cloud, 50, 0.008, generator);

    if (plane.inliers.size() < 3)
    {
        std::cerr << "RANSAC failed.\n";
        return 1;
    }

    // Normal pointing towards the camera
    Eigen::Vector3d n = plane.normal;

    if (n.z() > 0.0)
        n = -n;

    // X axis projected onto the detected plane
    Eigen::Vector3d axisX = Eigen::Vector3d::UnitX();
    axisX = (axisX - axisX.dot(n) * n).normalized();

    // Y axis perpendicular to X and the normal
    Eigen::Vector3d axisY = n.cross(axisX);

    // Origin on the detected plane
    Eigen::Vector3d origin = -plane.d * plane.normal;

    // Transform cloud into the plane coordinate system
    std::vector<Eigen::Vector3d> heightCloud;
    heightCloud.reserve(cloud.size());

    for (const auto& point : cloud)
    {
        Eigen::Vector3d q = point - origin;

        double x = q.dot(axisX);
        double y = q.dot(axisY);
        double h = q.dot(n);

        heightCloud.emplace_back(x, y, h);
    }

    SaveCSV(heightCloud, "real_height.csv");


    // Separate plane and remaining points
    std::vector<bool> isPlane(cloud.size(), false);

    for (int index : plane.inliers)
        isPlane[index] = true;

    std::vector<Eigen::Vector3d> planePoints;
    std::vector<Eigen::Vector3d> remainingPoints;

    for (size_t i = 0; i < cloud.size(); ++i)
    {
        if (isPlane[i])
            planePoints.push_back(cloud[i]);
        else
            remainingPoints.push_back(cloud[i]);
    }

    // Save results
    SaveCSV(planePoints, "real_plane.csv");
    SaveCSV(remainingPoints, "real_nonplane.csv");

    // Print results
    std::cout << "\n--- REAL PLANE RECONSTRUCTION ---\n";
    std::cout << "Global PCA normal: "
            << globalPCA.normal.transpose() << "\n";

    std::cout << "RANSAC + PCA normal: "
            << plane.normal.transpose() << "\n";

    std::cout << "Plane offset d: " << plane.d << " m\n";
    std::cout << "Plane points: " << planePoints.size() << "\n";
    std::cout << "Remaining points: " << remainingPoints.size() << "\n";

    std::cout << "Plane RMSE: "
            << CalcRMSE(planePoints, plane.normal, plane.d) * 1000
            << " mm\n";

    std::cout << "Resolution: "
              << depth.cols << " x " << depth.rows << "\n";

    std::cout << "Valid 3D points: "
              << cloud.size() << "\n";

    return 0;
}