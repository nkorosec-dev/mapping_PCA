#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <string>


int main()
{
    cv::VideoCapture camera(0);
    cv::Mat frame, blurred, hsv, mask;

    while (true)
    {
        camera.read(frame);

        cv::GaussianBlur(frame, blurred, cv::Size(5,5), 1);

        cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(35, 60, 40), cv::Scalar(85, 255, 255), mask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for(const auto &contour : contours) 
    
        {

            std::vector<cv::Point> approx;

            double area = cv::contourArea(contour);
            
            if(area < 1000)
            {
                continue;
            }
            else
            {
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{contour}, 0, cv::Scalar(0, 0, 255), 2);

                double perimeter = cv::arcLength(contour, true);

                double circularity = (4 * CV_PI * area) / std::pow(perimeter, 2);

                std::string shape;

                std::cout << "circularity: " << circularity << std::endl;

                cv::approxPolyDP(contour, approx, 0.03 * perimeter, true);

                std::cout << "vertices: " << approx.size() << std::endl;

   
                if(approx.size() == 3)
                {
                    shape = "trikotnik";
                }
                else if(approx.size() == 4)
                {
                    shape = "Kvadrat";
                }
                else if(approx.size() > 4 && circularity > 0.85)
                {
                    shape = "krog";
                }
                else
                {
                    shape = "neznano";
                }
 

                cv::Rect box = cv::boundingRect(contour);

                cv::putText(frame, shape, cv::Point(box.x, box.y -10), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }

            
        }

        cv::imshow("zelena detekcija", frame);

        if(cv::waitKey(1) == 'q')
        {
            break;
        }
        
    }
    

    return 0;
}