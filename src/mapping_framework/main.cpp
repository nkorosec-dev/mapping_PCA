#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <numbers>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>

struct Result
{
    double angleError;
    double rmse;
    double centroidError;
};

std::vector<Eigen::Vector3d> PointGen(double step, int N);
void TransformPoints(std::vector<Eigen::Vector3d> &points, const Eigen::Matrix3d &R, const Eigen::Vector3d &T);
void SaveCSV(std::vector<Eigen::Vector3d> &points, const std::string filename);
void AddGausNoise(std::vector<Eigen::Vector3d> &points, double sigma, std::mt19937 &generator);
Eigen::Vector3d CalcCentroid(const std::vector<Eigen::Vector3d> &points);
Eigen::Matrix3d CalcCovariance(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &centroid);
double CalcRMSE(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &normal, double d);
Result RunExperiment(const std::vector<Eigen::Vector3d> &reference, const Eigen::Vector3d &trueNormal, double sigma, std::mt19937 &generator, bool saveData = false);
void MonteCarloNoise(const std::vector<Eigen::Vector3d>& reference, const Eigen::Vector3d& trueNormal, const std::vector<double>& sigmas, int iterations, std::mt19937& generator);


int main()
{
    const double step = 0.05; // v m
    const int N = 40;

    double rx = 6.1 * std::numbers::pi / 180.0;
    double ry = 12.3 * std::numbers::pi / 180.0;

    auto generator = std::mt19937(42);

    Eigen::Vector3d trans(0.02, 0.13, 0.07);

    auto points = PointGen(step, N);

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix();

    Eigen::Matrix3d R = Ry * Rx;

    TransformPoints(points, R, trans);

    auto reference = points;

    Eigen::Vector3d trueNormal = R * Eigen::Vector3d::UnitZ(); //known

    SaveCSV(reference, "reference.csv");

    Result single = RunExperiment(
        reference, trueNormal, 0.1, generator, true
    );

    std::vector<double> sigmas = {0.0, 0.005, 0.01, 0.03, 0.05, 0.1};

    MonteCarloNoise(reference, trueNormal, sigmas, 100, generator);

    return 0;
    }

std::vector<Eigen::Vector3d> PointGen(double step, int N)
{
    std::vector<Eigen::Vector3d> points;

    points.reserve((N+1)*(N+1));

    for (int i = 0; i <= N; i++)
    {
        for(int j = 0; j <= N; j++)
        {   
            Eigen::Vector3d point(i*step, j*step, 0.0);

            points.push_back(point);
        }
    }
    return points;
}

void TransformPoints(std::vector<Eigen::Vector3d> &points, const Eigen::Matrix3d &R, const Eigen::Vector3d &T)
{
    for(auto &point : points)
    {
        point = R * point + T;
    }
}

void SaveCSV(std::vector<Eigen::Vector3d> &points, const std::string filename)
{
    std::ofstream file(filename);

    file << "x,y,z\n";

    for(const auto &point : points)
    {
        file << point.x() << "," << point.y() << "," << point.z() << "\n";
    }

}

void AddGausNoise(std::vector<Eigen::Vector3d> &points, double sigma, std::mt19937 &generator)
{
    std::normal_distribution<double> noise(0.0, sigma);

    for(auto &point : points)
    {
        point.x() += noise(generator);
        point.y() += noise(generator);
        point.z() += noise(generator);        
    }

}

Eigen::Vector3d CalcCentroid(const std::vector<Eigen::Vector3d> &points)
{
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    for(const auto &point : points)
    {
        centroid += point;
    }

    return centroid/static_cast<double>(points.size()); //povprecni polozaj vseh tock

}

Eigen::Matrix3d CalcCovariance(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &centroid)
{   
    Eigen::Matrix3d C = Eigen::Matrix3d::Zero();

    for(const auto &point : points)
    {
        Eigen::Vector3d q = point - centroid; //q pove koliko je posamezna tocka odmaknjena od tezisca

        C += q * q.transpose(); //zunanji produkt - kolk ima posamezna tocka prispevka
    }

    C /= static_cast<double>(points.size()); //kovariancna matrika

    return C;
}

double CalcRMSE(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &normal, double d)
{
    double sum = 0;

    for (const auto &point : points)
    {
        double error = normal.dot(point) + d;
        sum += error*error;
    }
    
    return std::sqrt(sum/ static_cast<double>(points.size()));
}

Result RunExperiment(const std::vector<Eigen::Vector3d> &reference, const Eigen::Vector3d &trueNormal, double sigma, std::mt19937 &generator, bool saveData)
{
    auto measured = reference;

    AddGausNoise(measured, sigma, generator);

    if(saveData)
    {
        SaveCSV(measured, "measured.csv");
    }


    auto estimated = CalcCentroid(measured);
    auto actual = CalcCentroid(reference);

    Eigen::Matrix3d C = CalcCovariance(measured, estimated);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
    Eigen::Vector3d normal = solver.eigenvectors().col(0); //najmanj razpersenosti

    double dot = std::abs(trueNormal.dot(normal));
    dot = std::clamp(dot, 0.0, 1.0);

    double angleError = std::acos(dot) * 180.0 / std::numbers::pi;

    double d = -normal.dot(estimated);

    double rmse = CalcRMSE(measured, normal, d);

    double centroidError = (estimated - actual).norm();

    if (saveData)
    {
        std::ofstream file("plane_fit.csv");

        file << "nx,ny,nz,d,angle_error_deg,rmse,centroid_error\n";

        file << normal.x() << ","
            << normal.y() << ","
            << normal.z() << ","
            << d << ","
            << angleError << ","
            << rmse << ","
            << centroidError << "\n";
    }

    return {angleError, rmse, centroidError};
}

void MonteCarloNoise(const std::vector<Eigen::Vector3d>& reference, const Eigen::Vector3d& trueNormal, const std::vector<double>& sigmas, int iterations, std::mt19937& generator)
{
    if (iterations < 2)
    {
        std::cerr << "Monte Carlo requires at least 2 iterations.\n";
        return;
    }

    std::ofstream file("monte_carlo_noise.csv");

    file << "sigma,mean_angle,std_angle,mean_rmse,mean_centroid\n";

    for (double sigma : sigmas)
    {
        double sumAngle = 0.0;
        double sumAngleSquared = 0.0;
        double sumRMSE = 0.0;
        double sumCentroid = 0.0;

        for (int i = 0; i < iterations; i++)
        {
            Result result = RunExperiment(
                reference, trueNormal, sigma, generator
            );

            sumAngle += result.angleError;
            sumAngleSquared += result.angleError * result.angleError;

            sumRMSE += result.rmse;
            sumCentroid += result.centroidError;
        }

        double meanAngle = sumAngle / iterations;
        double meanRMSE = sumRMSE / iterations;
        double meanCentroid = sumCentroid / iterations;

        // Vzorcna varianca kotne napake
        double variance =
            (sumAngleSquared - iterations * meanAngle * meanAngle)
            / (iterations - 1);

        double stdAngle = std::sqrt(std::max(0.0, variance));

        std::cout << "\nSigma: " << sigma << " m\n";
        std::cout << "Mean angle: " << meanAngle << " deg\n";
        std::cout << "Std angle: " << stdAngle << " deg\n";
        std::cout << "Mean RMSE: " << meanRMSE << " m\n";

        file << sigma << ","
             << meanAngle << ","
             << stdAngle << ","
             << meanRMSE << ","
             << meanCentroid << "\n";
    }
}