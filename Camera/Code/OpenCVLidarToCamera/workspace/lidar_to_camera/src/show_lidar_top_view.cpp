#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview()
{
    std::vector<LidarPoint> lidarPoints;
    readLidarPts("../dat/C51_LidarPts_0000.dat", lidarPoints); // reading lidar points

    cv::Size worldSize(10.0, 20.0); // width and height of sensor field in m, as seen from top view, height 20m is in driving direction
    cv::Size imageSize(1000, 2000); // corresponding top view image in pixel, 2000 pixels is in driving direction
    // 10 mt to be mapped to 1000 pixels. and 20mt to 2000 pixels

    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0)); // 8bit unsigned int, 3 channels, black image (0, 0, 0) 

    // plot Lidar points into image (looping over all the lidar points)
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        float xw = (*it).x; // world position in m with x facing forward from sensor, *it
        float yw = (*it).y; // world position in m with y facing left from sensor

        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height; // xw is in meters, y is the image coordinate system. Then scaling happens imageSize.height / worldSize.height)
        int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;
        // Finally we get top view image. 
        //cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        
        // TODO: 
        // 1. Change the color of the Lidar points such that 
        // X=0.0m corresponds to red while X=20.0m is shown as green.
        // 2. Remove all Lidar points on the road surface while preserving 
        // measurements on the obstacles in the scene.
        // STUDENT EXERCISE 2
        float zw = (*it).z; // world position in m with y facing left from sensor, height above road surface. 0.0 will be height of lidar sensor, upward is positive. 
        if(zw > -1.40){ // -1.55mt downward is road.
        // EOF STUDENT EXERCISE 2            

            // STUDENT EXERCISE 1
            float val = it->x; // x value in driving direction 
            float maxVal = worldSize.height; // worldSize.height = 20mt it is the maximum value 
            int red = min(255, (int)(255 * abs((val - maxVal) / maxVal))); //(int) converts the value to integer, When we are zero meters away from sensor, it should show red
            int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal)))); // We make the green channel exactly opposite to red channel
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, red), -1); // 5 is about, 1 lidar point having 5 pixels. Green and Red values for a lidar point changes with its position in the worldSize
            // EOF STUDENT EXERCISE 1
        }

    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0)); //topviewImg is 20mth in height, painting a blue line running horizontal in the image
    }

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    showLidarTopview();
}