
#include <iostream>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "lidarData.hpp"


// using namespace std;

// remove Lidar points based on min. and max distance in X, Y and Z
void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, const float minX, const float maxX, const float maxY, const float minZ, const float maxZ, const float minR)
{
    std::vector<LidarPoint> newLidarPts; 
    // Iterating over all the lidat points came throubh lidarPoints reference. Then, applying if condition to remove the lidat points out of the limits
    for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it) {
       // Check if Lidar point is outside of boundaries 
       if( (*it).x>=minX && (*it).x<=maxX && (*it).z>=minZ && (*it).z<=maxZ && (*it).z<=0.0 && abs((*it).y)<=maxY && (*it).r>=minR )  // Check if Lidar point is outside of boundaries
       {
           newLidarPts.push_back(*it);
       }
    }

    lidarPoints = newLidarPts;
}


// Load Lidar points from a given location and store them in a vector
void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, const std::string filename)
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    unsigned long num = 1000000;
    // float *data = (float*)malloc(num*sizeof(float));
    float *data = static_cast<float*>(malloc(num * sizeof(float)));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;
    
    // load point cloud
    FILE *stream;
    stream = fopen (filename.c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;
 
    for (int32_t i=0; i<num; i++) 
    {
        LidarPoint lpt;
        lpt.x = *px; 
        lpt.y = *py; 
        lpt.z = *pz; 
        lpt.r = *pr;
        lidarPoints.push_back(lpt);
        px+=4; 
        py+=4; 
        pz+=4; 
        pr+=4;
    }
    fclose(stream);
}

// Showing lidar points in a image. 
void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    const int scale = imageSize.height / worldSize.height ;
    const int halfWidth = imageSize.width / 2 ;

    // plot Lidar points into image
    for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
        // const float xw = (*it).x; // world position in m with x facing forward from sensor
        // const float yw = (*it).y; // world position in m with y facing left from sensor

        // const int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        // const int x = (-yw * imageSize.height / worldSize.height) + imageSize.width / 2;

        const float xw = static_cast<float>(it->x) ; // world position in m with x facing forward from sensor
        const float yw = static_cast<float>(it->y) ; // world position in m with y facing left from sensor

        const int y = static_cast<int>((-xw * scale) + imageSize.height) ;
        const int x = static_cast<int>((-yw * scale) + halfWidth) ;

        cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
    }

    // plot distance markers
    constexpr float lineSpacing = 2.0; // gap between distance markers
    const int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        // const int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        const int y = static_cast<int>((-(i * lineSpacing) * scale) + imageSize.height) ;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    // std::string windowName = "Top-View Perspective of LiDAR data";
    constexpr char windowName[]{ "Top-View Perspective of LiDAR data" };

    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    if(bWait)
    {
        // cv::waitKey(0); // wait for key to be pressed
    }
}

void showLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidarPoints, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT, cv::Mat *extVisImg)
{
    // init image for visualization
    cv::Mat visImg; 
    if(extVisImg==nullptr)
    {
        visImg = img.clone();
    } else 
    {
        visImg = *extVisImg;
    }

    cv::Mat overlay = visImg.clone();

    // find max. x-value
    double maxVal = 0.0; 
    // for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it)
    for (auto it = std::begin(lidarPoints); it!= std::end(lidarPoints); ++it) 
    {
        maxVal = maxVal < it->x ? it->x : maxVal; // If maxVal is less than it->x than use first value after ?. If not use value after :.
    }

    cv::Mat X(4,1,cv::DataType<double>::type);
    cv::Mat Y(3,1,cv::DataType<double>::type);
    for(auto it=lidarPoints.begin(); it!=lidarPoints.end(); ++it) {

            X.at<double>(0, 0) = it->x;
            X.at<double>(1, 0) = it->y;
            X.at<double>(2, 0) = it->z;
            X.at<double>(3, 0) = 1;

            Y = P_rect_xx * R_rect_xx * RT * X;
            cv::Point pt;
            pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
            pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

            // float val = it->x;
            // int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
            // int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));

            const float val = static_cast<float>(it->x) ;
            const int red = std::min(255, static_cast<int>(255 * abs((val - maxVal) / maxVal))) ;
            const int green = std::min(255, static_cast<int>(255 * (1 - abs((val - maxVal) / maxVal)))) ;
            
            cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
    }

    constexpr float opacity = 0.6;
    cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

    // return augmented image or wait if no image has been provided
    if (extVisImg == nullptr)
    {
        // std::string windowName = "LiDAR data on image overlay";
        constexpr char windowName[]{ "LiDAR data on image overlay" };
        cv::namedWindow( windowName, 3 );
        cv::imshow( windowName, visImg );
        // cv::waitKey(0); // wait for key to be pressed
    }
    else
    {
        extVisImg = &visImg;
    }
}