#pragma once

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <string>

struct Result
{
    double angleError;
    double rmse;
    double centroidError;
};

struct PlaneModel
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    double d = 0.0;
    std::vector<int> inliers;
};

std::vector<Eigen::Vector3d> PointGen(double step, int N);

std::vector<Eigen::Vector3d>  CubeGen(double step, int N, double cx, double cy);

void TransformPoints(
    std::vector<Eigen::Vector3d> &points, 
    const Eigen::Matrix3d &R, 
    const Eigen::Vector3d &T);

void SaveCSV(
    const std::vector<Eigen::Vector3d> &points, 
    const std::string filename);

void AddGausNoise(
    std::vector<Eigen::Vector3d> &points, 
    double sigma, std::mt19937 &generator);

Eigen::Vector3d CalcCentroid(
    const std::vector<Eigen::Vector3d> &points);

Eigen::Matrix3d CalcCovariance(
    const std::vector<Eigen::Vector3d> &points, 
    const Eigen::Vector3d &centroid);

PlaneModel FitPlanePCA(
    const std::vector<Eigen::Vector3d> &points);

double CalcRMSE(
    const std::vector<Eigen::Vector3d> &points, 
    const Eigen::Vector3d &normal, 
    double d);

PlaneModel RansacPlane(
    const std::vector<Eigen::Vector3d> &points, 
    int iterations, double threshold, 
    std::mt19937 &generator);

Result RunExperiment(
    const std::vector<Eigen::Vector3d> &reference, 
    const Eigen::Vector3d &trueNormal, 
    double sigma, std::mt19937 &generator, 
    bool saveData = false);

void MonteCarloNoise(
    const std::vector<Eigen::Vector3d>& reference, 
    const Eigen::Vector3d& trueNormal, 
    const std::vector<double>& sigmas, int iterations, 
    std::mt19937 &generator);
