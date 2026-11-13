#include "mapping.hpp"

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry>
#include <numbers>
#include <algorithm>
#include <cmath>


std::vector<Eigen::Vector3d> PointGen(double step, int N)
{
    std::vector<Eigen::Vector3d> points;

    points.reserve((N + 1) * (N + 1));

    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= N; j++)
        {
            Eigen::Vector3d point(i * step, j * step, 0.0);
            points.push_back(point);
        }
    }

    return points;
}


std::vector<Eigen::Vector3d>  CubeGen(double step, int N, double cx, double cy)
{
    std::vector<Eigen::Vector3d> points;

    double side = N * step;
    double x0 = cx - side / 2.0;
    double y0 = cy - side / 2.0;

    for(int i =  0; i <= N; i++)
    {
        for(int j = 0; j <= N; j++)
        {
            for(int k = 0; k <= N; k++)
            {
                if(k > 0 && (k == N || i == 0 || i == N || j == 0 || j == N))
                {
                    Eigen::Vector3d point(x0 + i * step, y0 + j * step, k * step);
                    points.push_back(point);                
                }
            }
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

void SaveCSV(const std::vector<Eigen::Vector3d> &points, const std::string filename)
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

    if(sigma == 0.0)
    {
        return;
    }

    if(sigma < 0.0)
    {
        throw std::invalid_argument("SIGMA MUST BE A POSITIVE NUUMBER!");
    }

    std::normal_distribution<double> noise(0.0, sigma); 

    for(auto &point : points) //dodam pseudoreandom sum
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

        C += q * q.transpose(); //zunanji produkt - kolk ima posamezna tocka prispevka, varianca in kovarianca
    
    }    
    C /= static_cast<double>(points.size()); //kovariancna matrika, poveprecje prispevka veh tock

    return C;
}

PlaneModel FitPlanePCA(const std::vector<Eigen::Vector3d> &points)
{
    if (points.size() < 3)
        throw std::invalid_argument("At least 3 points are required");

    Eigen::Vector3d centroid = CalcCentroid(points);
    Eigen::Matrix3d C = CalcCovariance(points, centroid);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);

    PlaneModel plane;

    plane.normal = solver.eigenvectors().col(0);
    plane.d = -plane.normal.dot(centroid);

    return plane;
}


double CalcRMSE(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &normal, double d)
{
    double sum = 0;

    for (const auto &point : points)
    {
        double error = normal.dot(point) + d; //skalarni produkt normale in tocke - pove kam kazze vektor glede na drugega
        sum += error*error; //kvadraten odmik - navadno povprecje bi dobili 0 - kvadrirano zato potem korenimo
    }
    
    return std::sqrt(sum/ static_cast<double>(points.size())); //Root Mean Squared Error.
}


PlaneModel RansacPlane(const std::vector<Eigen::Vector3d> &points, int iterations, double threshold, std::mt19937 &generator)
{
    PlaneModel best;

    if(points.size() < 3)
        return best;

    std::uniform_int_distribution<int> randIndex(0, static_cast<int>(points.size()) - 1);

    for (int i = 0; i < iterations; i++)
    {
        int a = randIndex(generator);
        int b = randIndex(generator);
        int c = randIndex(generator);

        if (a == b || a == c || b == c)
            continue;
        
        Eigen::Vector3d v1 = points[b] - points[a];
        Eigen::Vector3d v2 = points[c] - points[a];

        Eigen::Vector3d normal = v1.cross(v2);  

         if (normal.norm() < 1e-10)
            continue;

        normal.normalize();

        double d = -normal.dot(points[a]);

        std::vector<int> inliers;

        for (int j = 0; j < static_cast<int>(points.size()); j++)
        {
            double distance = std::abs(normal.dot(points[j]) + d);

            if (distance < threshold)
                inliers.push_back(j);
        }

        if (inliers.size() > best.inliers.size())
        {
            best.normal = normal;
            best.d = d;
            best.inliers = inliers;
        }
    }

    if (best.inliers.size() >= 3)
    {
        std::vector<Eigen::Vector3d> planePoints;

        for (int index : best.inliers)
            planePoints.push_back(points[index]);

        PlaneModel refined = FitPlanePCA(planePoints);

        best.normal = refined.normal;
        best.d = refined.d;

        best.inliers.clear();

        for (int j = 0; j < static_cast<int>(points.size()); j++)
        {
            double distance =
                std::abs(best.normal.dot(points[j]) + best.d);

            if (distance < threshold)
                best.inliers.push_back(j);
        }
    }

    return best;
}


Result RunExperiment(const std::vector<Eigen::Vector3d> &reference, const Eigen::Vector3d &trueNormal, double sigma, std::mt19937 &generator, bool saveData)
{
    auto measured = reference; //kopija

    AddGausNoise(measured, sigma, generator); //dodan šum

    PlaneModel plane = FitPlanePCA(measured);

    double dot = std::abs(trueNormal.dot(plane.normal)); //skalarni produkt enotskih normal
    dot = std::clamp(dot, 0.0, 1.0); //omejitev za acos()

    double angleError = std::acos(dot) * 180.0 / std::numbers::pi; //kot med normalama

    double rmse = CalcRMSE(measured, plane.normal, plane.d); 
    
    Eigen::Vector3d estimated = CalcCentroid(measured);
    Eigen::Vector3d actual = CalcCentroid(reference);

    double centroidError = (estimated - actual).norm();

    if (saveData)
    {
        SaveCSV(measured, "measured.csv");

        std::ofstream file("plane_fit.csv");

        file << "nx,ny,nz,d,angle_error_deg,rmse,centroid_error\n";

        file << plane.normal.x() << ","
             << plane.normal.y() << ","
             << plane.normal.z() << ","
             << plane.d << ","
             << angleError << ","
             << rmse << ","
             << centroidError << "\n";
    }


    return {angleError, rmse, centroidError};
}

void MonteCarloNoise(const std::vector<Eigen::Vector3d>& reference, const Eigen::Vector3d& trueNormal, const std::vector<double>& sigmas, int iterations, std::mt19937& generator)
{
    std::ofstream file("monte_carlo_noise.csv");

    file << "sigma,mean_angle,std_angle,mean_rmse,mean_centroid\n";

    for (double sigma : sigmas) // spreminja gaussov sum
    {
        double sumAngle = 0.0;
        double sumAngleSquared = 0.0;
        double sumRMSE = 0.0;
        double sumCentroid = 0.0;

        for (int i = 0; i < iterations; i++) // rekonstrukcija pri istem sigma in drugi realiziaciji suma
        {
            Result result = RunExperiment(reference, trueNormal, sigma, generator, false);
            // Zbiranje napak vseh ponovitev
            sumAngle += result.angleError;
            sumAngleSquared += result.angleError * result.angleError;
            sumRMSE += result.rmse;
            sumCentroid += result.centroidError;
        }
        // Povprecne napake rekonstrukcije
        double meanAngle = sumAngle / iterations;
        double meanRMSE = sumRMSE / iterations;
        double meanCentroid = sumCentroid / iterations;

        // Vzorcna varianca kotne napake
        double variance = (sumAngleSquared - iterations * meanAngle * meanAngle)/ (iterations - 1);

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