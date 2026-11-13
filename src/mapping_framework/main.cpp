#include "mapping.hpp"

#include <iostream>
#include <Eigen/Geometry>
#include <numbers>
#include <algorithm>
#include <cmath>

int main()
{
    const double step = 0.01;
    const int N = static_cast<int>(std::round(2.0 / step));
    const int cubeN = static_cast<int>(std::round(0.4 / step));

    std::mt19937 generator(std::random_device{}());

    const double rx = 6.1 * std::numbers::pi / 180.0;
    const double ry = 12.3 * std::numbers::pi / 180.0;

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Matrix3d R = Ry * Rx;
    Eigen::Vector3d T(0.02, 0.13, 0.07);

    auto reference = PointGen(step, N);
    TransformPoints(reference, R, T);

    Eigen::Vector3d trueNormal = R * Eigen::Vector3d::UnitZ();

    SaveCSV(reference, "reference.csv");

    RunExperiment(reference, trueNormal, 0.1, generator, true);

    std::vector<double> sigmas = {0, 0.005, 0.01, 0.03, 0.05, 0.1, 0.2};
    MonteCarloNoise(reference, trueNormal, sigmas, 100, generator);

    auto cube = CubeGen(step, cubeN, 1.3, 1.3);
    TransformPoints(cube, R, T);

    auto scene = reference;
    scene.insert(scene.end(), cube.begin(), cube.end());

    AddGausNoise(scene, 0.01, generator);
    SaveCSV(scene, "scene.csv");

    PlaneModel globalPlane = FitPlanePCA(scene);
    PlaneModel ransacPlane = RansacPlane(scene, 300, 0.03, generator);

    if (ransacPlane.inliers.size() < 3)
    {
        std::cerr << "RANSAC failed.\n";
        return 1;
    }

    auto angleError = [&](const Eigen::Vector3d& normal)
    {
        double dot = std::clamp(std::abs(normal.dot(trueNormal)), 0.0, 1.0);
        return std::acos(dot) * 180.0 / std::numbers::pi;
    };

    std::vector<bool> isInlier(scene.size(), false);

    for (int index : ransacPlane.inliers)
        isInlier[index] = true;

    std::vector<Eigen::Vector3d> planePoints, objectPoints;

    for (size_t i = 0; i < scene.size(); i++)
    {
        if (isInlier[i])
            planePoints.push_back(scene[i]);
        else
            objectPoints.push_back(scene[i]);
    }

    SaveCSV(planePoints, "ransac_inliers.csv");
    SaveCSV(objectPoints, "ransac_outliers.csv");

    std::cout << "\n--- PLANE + CUBE ---\n";
    std::cout << "Total points: " << scene.size() << "\n";
    std::cout << "Plane points: " << planePoints.size() << "\n";
    std::cout << "Object points: " << objectPoints.size() << "\n";
    std::cout << "Global PCA error: " << angleError(globalPlane.normal) << " deg\n";
    std::cout << "RANSAC error: " << angleError(ransacPlane.normal) << " deg\n";
    std::cout << "RANSAC RMSE: "
              << CalcRMSE(planePoints, ransacPlane.normal, ransacPlane.d) * 1000
              << " mm\n";

    return 0;
}