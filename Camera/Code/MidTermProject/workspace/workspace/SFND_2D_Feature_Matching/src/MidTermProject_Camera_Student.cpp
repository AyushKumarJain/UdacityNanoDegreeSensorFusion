/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

// using namespace std;
inline const bool isValidDescriptorDetectorCombo(const std::string descriptor, const std::string detector) 
{
    return !((descriptor.compare("AKAZE") == 0 && detector.compare("AKAZE") != 0) || (descriptor.compare("ORB") == 0 && detector.compare("SIFT") == 0));
}

std::vector<TimingInfo> initializeTimingInfoVector(void) 
{
    const std::vector<std::string> detectorTypes{ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE",  "SIFT" };
    const std::vector<std::string> descriptorTypes{ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" };
    const std::vector<std::string> matcherTypes{ "MAT_BF" };
    const std::vector<std::string> selectorTypes{ "SEL_KNN" };

    std::vector<TimingInfo> info;

    for (auto detectorType : detectorTypes) 
    {
        for (auto descriptorType : descriptorTypes) 
        {
            for (auto matcherType : matcherTypes) 
            {
                for (auto selectorType : selectorTypes) 
                {

                    if (!isValidDescriptorDetectorCombo(descriptorType, detectorType)) { continue; }

                    info.push_back(TimingInfo(detectorType, descriptorType, matcherType, selectorType));
                }
            }
        }
    }

    return info;
}

void createCSVOutputFile(std::vector<TimingInfo> &timingInfo) 
{
    constexpr char COMMA[]{ ", " };
    constexpr char csvName[]{ "../report/Ayush_Jain_Midterm_Project.csv" };

    std::cout << "Writing output file: " << csvName << std::endl;
    std::ofstream csvStream{ csvName };

    csvStream << "Name: Ayush Jain" << std::endl << "Date: 2020-04-27" << std::endl << std::endl;

    csvStream << "IMAGE NO." << COMMA;
    csvStream << "DETECTOR TYPE" << COMMA;
    csvStream << "DESCRIPTOR TYPE" << COMMA;
    csvStream << "TOTAL KEYPOINTS" << COMMA;
    csvStream << "KEYPOINTS ON VEHICLE" << COMMA;
    csvStream << "DETECTOR ELAPSED TIME" << COMMA;
    csvStream << "DESCRIPTOR ELAPSED TIME" << COMMA;
    csvStream << "MATCHED KEYPOINTS" << COMMA;
    csvStream << "MATCHER ELAPSED TIME";
    csvStream << std::endl;

    for (auto &info : timingInfo) {
        for (int index{ 0 }; index < 10; index++) {
            csvStream << index << COMMA;
            csvStream << info.detectorType << COMMA;
            csvStream << info.descriptorType << COMMA;
            csvStream << info.ptsPerFrame.at(index) << COMMA;
            csvStream << info.pointsLeftOnImg.at(index) << COMMA;
            csvStream << info.detElapsedTime.at(index) << COMMA;
            csvStream << info.descElapsedTime.at(index) << COMMA;
            csvStream << info.matchedPts.at(index) << COMMA;
            csvStream << info.matchElapsedTime.at(index) << std::endl;
        }

        csvStream << std::endl;
    }

    csvStream.close();
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    std::vector<TimingInfo> timingInfo{ initializeTimingInfoVector() };

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    const std::string dataPath = "../";

    // camera
    // It gives a sequence of 10 images, saved in images folder. Compared to 1st image, vehile in last image is close to our vehicle.
    const std::string imgBasePath = dataPath + "images/";
    const std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    const std::string imgFileType = ".png";
    constexpr int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    constexpr int imgEndIndex = 9;   // last file index to load
    constexpr int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
    constexpr int totalImages{ imgEndIndex - imgStartIndex };

    // misc
    constexpr int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time, // We will implement ring buffer, it will hold keypoint matches, lidar points, it will be stored here. 
    bool bVis = false;            // visualize results (visualisation flag)

    /* MAIN LOOP OVER ALL IMAGES */
    for (auto &info : timingInfo) 
    {
        dataBuffer.clear();
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
        {   
            DetectionData data;

            std::cout << "===== DETAILS FOR IMAGE: " << (imgIndex + 1) << " =====\n" << std::endl;
            std::cout << "DETECTOR Type := " << info.detectorType << std::endl;
            std::cout << "DESCRIPTOR Type := " << info.descriptorType << std::endl;
            std::cout << "MATCHER Type := " << info.matcherType << std::endl;
            std::cout << "SELECTOR Type := " << info.selectorType << std::endl;
            std::cout << std::endl;
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            std::ostringstream imgNumber;
            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
            const std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType; // it gives absolute of the image that we are loading

            // load image from file and convert to grayscale
            const cv::Mat img = cv::imread(imgFullFilename);
            cv::Mat imgGray;
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY); 

            //// STUDENT ASSIGNMENT
            //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

            // push image into data frame buffer
            DataFrame frame;
            frame.cameraImg = imgGray; // Adding first image to frame, an object of DataFrame
            if (dataBuffer.size() == dataBufferSize) 
                dataBuffer.erase(std::begin(dataBuffer)); 
            dataBuffer.push_back(frame);
            //// EOF STUDENT ASSIGNMENT

            std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;

            /* DETECT IMAGE KEYPOINTS */

            // extract 2D keypoints from current image
            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image, It will all the keypoints detected in the image.
            // string detectorType = "SHITOMASI";

            //// STUDENT ASSIGNMENT
            //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
            //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
            
            // Here we choose Keypoint detector type, either Intensity gradient based (SHITOMASI, HARRIS, SIFT, SURF) Or binary (FAST, BRISK, ORB, AKAZE)
            if (info.detectorType.compare("SHITOMASI") == 0)
            {
                data = detKeypointsShiTomasi(keypoints, imgGray, false); // data is Extra
            }
            else if (info.detectorType.compare("HARRIS") == 0){
                 data = detKeypointsHarris(keypoints, imgGray, false); // data is Extra
            } else{
                data = detKeypointsModern(keypoints, imgGray, info.detectorType, false); // data is Extra
            }

            info.ptsPerFrame.at(imgIndex) = data.numKeyPoints; // Extra
            info.detElapsedTime.at(imgIndex) = data.elapsedTime; // Extra
             //// EOF STUDENT ASSIGNMENT

            //// STUDENT ASSIGNMENT
            //// TASK MP.3 -> only keep keypoints on the preceding vehicle

            // only keep keypoints on the preceding vehicle
            constexpr bool bFocusOnVehicle = true;
            const cv::Rect vehicleRect(535, 180, 180, 150); // Defines a rectange that enclose directly proceeding vehicle. We will keep keypoints which are in the bounding rectangle. 
            if (bFocusOnVehicle)
            {
                std::vector<cv::KeyPoint> retainedPoints;

                for (auto point : keypoints) 
                {
                    if (vehicleRect.contains(cv::Point2f(point.pt))) 
                        retainedPoints.push_back(point); 
                }

                keypoints = retainedPoints;

                info.pointsLeftOnImg.at(imgIndex) = keypoints.size();
                std::cout << std::endl;
            }

            //// EOF STUDENT ASSIGNMENT

            // optional : limit number of keypoints (helpful for debugging and learning)
            constexpr bool bLimitKpts = false; // Switch it on. This step helps to limit keypoints inside the bounding rectangle
            if (bLimitKpts)
            {
                constexpr int maxKeypoints = 50;

                if (info.detectorType.compare("SHITOMASI") == 0)
                { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                    keypoints.erase(std::begin(keypoints) + maxKeypoints, std::end(keypoints));
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                std::cout << " NOTE: Keypoints have been limited!" << std::endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            (std::end(dataBuffer) - 1)->keypoints = keypoints;
            std::cout << "#2 : DETECT KEYPOINTS done" << std::endl;

            /* EXTRACT KEYPOINT DESCRIPTORS */

            //// STUDENT ASSIGNMENT
            //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
            //// -> BRIEF, ORB, FREAK, AKAZE, SIFT( it has both keypoint detector and descriptor)

            cv::Mat descriptors;
            //string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
            data = descKeypoints((std::end(dataBuffer) - 1)->keypoints, (std::end(dataBuffer) - 1)->cameraImg, descriptors, info.descriptorType);
            info.descElapsedTime.at(imgIndex) = data.elapsedTime;
            //// EOF STUDENT ASSIGNMENT

            // push descriptors for current frame to end of data buffer
            (std::end(dataBuffer) - 1)->descriptors = descriptors; // dataBuffer.end(), when we use namespace std and std::end(dataBuffer) when we donot use

            std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl;
            // We need atleast two images to do descriptor matching. 
            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                /* MATCH KEYPOINT DESCRIPTORS */

                std::vector<cv::DMatch> matches;
                // string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                // string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
                // string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
                // matcherType, descriptorType and selectorType are defined in descriptorFamily method
                const std::string descriptorFamily = (info.descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY";

                //// STUDENT ASSIGNMENT
                //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                data = matchDescriptors((std::end(dataBuffer) - 2)->keypoints, (std::end(dataBuffer) - 1)->keypoints,
                                (std::end(dataBuffer) - 2)->descriptors, (std::end(dataBuffer) - 1)->descriptors,
                                matches, descriptorFamily, info.matcherType, info.selectorType);

                info.matchedPts.at(imgIndex) = data.numKeyPoints;
                info.matchElapsedTime.at(imgIndex) = data.elapsedTime;

                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                (std::end(dataBuffer) - 1)->kptMatches = matches;

                std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

                // visualize matches between current and previous image
                // bVis = true;
                if (bVis)
                {
                    cv::Mat matchImg = ((std::end(dataBuffer) - 1)->cameraImg).clone();
                    cv::drawMatches((std::end(dataBuffer) - 2)->cameraImg, (std::end(dataBuffer) - 2)->keypoints,
                                    (std::end(dataBuffer) - 1)->cameraImg, (std::end(dataBuffer) - 1)->keypoints,
                                    matches, matchImg,
                                    cv::Scalar::all(-1), cv::Scalar::all(-1),
                                    std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                    
                    std::string windowName = "Matching keypoints between two camera images";
                    cv::namedWindow(windowName, 7);
                    cv::imshow(windowName, matchImg);
                    std::cout << "Press key to continue to next image" << std::endl;
                    // cv::waitKey(0); // wait for key to be pressed
                }
                // bVis = false;
            } else {
                info.matchedPts.at(imgIndex) = info.matchElapsedTime.at(imgIndex) = 0;

            }

            std::cout << "\n================================\n" << std::endl;
            std::cout << "................................\n" << std::endl;

        } // eof loop over all images
    }

    createCSVOutputFile(timingInfo);

    return 0;
}
