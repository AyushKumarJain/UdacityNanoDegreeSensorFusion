
/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"  // Descriptor matching
#include "objectDetection2D.hpp" // Yolo
#include "lidarData.hpp"
#include "camFusion.hpp"

// using namespace std;

using std::end;
using std::begin;


// Student Code Start

inline const bool isValidDescriptorDetectorCombo(const std::string descriptor, const std::string detector) 
{
    return !((descriptor.compare("AKAZE") == 0 && detector.compare("AKAZE") != 0) ||
            (descriptor.compare("ORB") == 0 && detector.compare("SIFT") == 0));
}

// This function will give out a vector of TimingInfo object. Each TimingInfo object has different combination of detectorType, descriptorType, matcherType, selectorType
std::vector<TimingInfo> initializeTimingInfoVector(void) 
{
    const std::vector<std::string> detectorTypes{ "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE",  "SIFT" }; // 7
    const std::vector<std::string> descriptorTypes{ "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" }; // 6
    const std::vector<std::string> matcherTypes{ "MAT_BF" }; // 1
    const std::vector<std::string> selectorTypes{ "SEL_KNN" }; // 1

    std::vector<TimingInfo> info;

    for (auto detectorType : detectorTypes) {
        for (auto descriptorType : descriptorTypes) {
            for (auto matcherType : matcherTypes) {
                for (auto selectorType : selectorTypes) {

                    if (!isValidDescriptorDetectorCombo(descriptorType, detectorType)) { continue; }

                    info.push_back(TimingInfo(detectorType, descriptorType, matcherType, selectorType));
                }
            }
        }
    }

    return info;
}

void createCSVOutputFile(std::vector<TimingInfo> &timingInfo, const int numberOfImages) 
{
    constexpr char COMMA[]{ ", " };
    constexpr char csvName[]{ "../report/Ayush_Jain_Final_3D_Camera_Project.csv" };

    std::cout << "Writing output file: " << csvName << std::endl;
    std::ofstream csvStream{ csvName };

    csvStream << "Name: Ayush Jain" << std::endl << "Date: 2020-04-27" << std::endl << std::endl;

    csvStream << "IMAGE NO." << COMMA;
    csvStream << "DETECTOR TYPE" << COMMA;
    csvStream << "DESCRIPTOR TYPE" << COMMA;
    csvStream << "TOTAL KEYPOINTS" << COMMA;
    csvStream << "TTC LIDAR" << COMMA;
    csvStream << "TTC CAMERA" << COMMA;
    csvStream << std::endl;

    for (auto &info : timingInfo) {
        for (int index{ 0 }; index < numberOfImages; index++) {
            if (!info.enoughLidarOrCameraPointsDetected.at(index)) { continue; }

            csvStream << index << COMMA;
            csvStream << info.detectorType << COMMA;
            csvStream << info.descriptorType << COMMA;
            csvStream << info.ptsPerFrame.at(index) << COMMA;
            csvStream << info.ttcLidar.at(index) << COMMA;
            csvStream << info.ttcCamera.at(index) << std::endl;
        }
        csvStream << std::endl;
    }

    csvStream.close();
}

// Student Code End

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{   

    std::vector<TimingInfo> timingInfo = initializeTimingInfoVector();
    // initializeTimingInfoVector() function call will output a vector with dataType TimingInfo. The dataType will have combination of detectorType, descriptorType, matcherType, selectorType. The other variable in TiminigInfo (ptsPerFrame, matchedPts, ttcLidar, ttcCamera, enoughLidarOrCameraPointsDetected) will be filled after data processing.
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    const std::string dataPath = "../"; // relative to the location of executable, where is data, is specified by this 

    // camera
    const std::string imgBasePath = dataPath + "images/";
    const std::string imgPrefix = "KITTI/2011_09_26/image_02/data/000000"; // left camera, color, we are using color version here
    const std::string imgFileType = ".png";

    // Constexpr is used for int rather than strings.
    constexpr int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    constexpr int imgEndIndex = 18;   // last file index to load
    constexpr int imgStepWidth = 1;   // 1 shows we are using every image, Frame rate is currently 10 images/sec. We can increase or decrease with the help of this variable. Time between two instances can be increases with this variable. What will be effect on key point matching can be studied
    constexpr int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // object detection
    const std::string yoloBasePath = dataPath + "dat/yolo/";
    const std::string yoloClassesFile = yoloBasePath + "coco.names";
    const std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    const std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // Lidar
    const std::string lidarPrefix = "KITTI/2011_09_26/velodyne_points/data/000000";
    const std::string lidarFileType = ".bin";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    constexpr double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    constexpr int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results, when in need make it true below in the code

    /* MAIN LOOP OVER ALL IMAGES */
    constexpr size_t numImages = imgEndIndex - imgStartIndex;
    
    // The first for loop runs over different combination of detectorType, descriptorType, matcherType, selectorType, which are stored in vector form in timingInfo (an object of std::vector<TimingInfo>).
    for (auto &info : timingInfo) {
        dataBuffer.clear();

        std::cout << "===== SETTINGS: " << " =====" << std::endl;
        std::cout << "......................" << std::endl;
        std::cout << "DETECTOR Type := " << info.detectorType << std::endl; // info (a TimingInfo dataType) stored in timingInfo
        std::cout << "DESCRIPTOR Type := " << info.descriptorType << std::endl;
        std::cout << "MATCHER Type := " << info.matcherType << std::endl;
        std::cout << "SELECTOR Type := " << info.selectorType << std::endl;
        std::cout << "......................" << std::endl;
        std::cout << std::endl;
        
        // This for loop runs over all the images given as data. It starts from the first image (imgIndex=0) and goes till last image (imgEndIndex - imgStartIndex = 18). The jump to next image is decided by imgStepWidth 
        for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex+=imgStepWidth) // imgStepWidth = 1 currently
        {    
            // DetectionData is a data structure that stores numKeyPoints and elapsedTime for different combination of detectorType, descriptorType, matcherType, selectorType.
            DetectionData data;
            /* LOAD IMAGE INTO BUFFER */

            // assemble filenames for current index
            std::ostringstream imgNumber;
            imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex; // imgStartIndex = 0 is fixed. Also We have imgFillWidth = 4; no. of digits which make up the file index (e.g. img-0001.png)
            std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

            // load image from file 
            cv::Mat img = cv::imread(imgFullFilename);

            // push image into data frame buffer
            DataFrame frame; // DataFrame dataType has cameraImg, keypoints, descriptors, kptMatches, lidarPoints, boundingBoxes, bbMatches.
            frame.cameraImg = img;
            if (dataBuffer.size() == dataBufferSize) // size(), erase(), push_back() are all methods that can be used for a vector
                dataBuffer.erase(std::begin(dataBuffer)); 
            dataBuffer.push_back(frame); // dataBuffer is a vector of dataType DataFrame.

            // std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;


            /* DETECT & CLASSIFY OBJECTS */

            constexpr float confThreshold = 0.2;
            constexpr float nmsThreshold = 0.4;
            bVis = true;        
            // detectObjects((dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->boundingBoxes, confThreshold, nmsThreshold,
            //            yoloBasePath, yoloClassesFile, yoloModelConfiguration, yoloModelWeights, bVis); // Here dataBuffer boundingBoxes are added with the detected rectangles. it is basically output of this function 
            detectObjects((end(dataBuffer) - 1)->cameraImg, // Calling cameraImg and boundingBoxes part of frame (), stored in dataBuffer with other constant variables
                          (end(dataBuffer) - 1)->boundingBoxes, // boundingBoxes is a vector of boundingBox. Where, boundingBox has constants boxID, trackID, roi, classID, confidence and vectors lidarPoints, keypoints, kptMatches.
                          confThreshold,
                          nmsThreshold,
                          yoloBasePath,
                          yoloClassesFile,
                          yoloModelConfiguration,
                          yoloModelWeights,
                          bVis);
            // std::cout << "#2 : DETECT & CLASSIFY OBJECTS done" << std::endl;


            /* CROP LIDAR POINTS */

            // load 3D Lidar points from file
            const std::string lidarFullFilename = imgBasePath + lidarPrefix + imgNumber.str() + lidarFileType;
            std::vector<LidarPoint> lidarPoints;
            loadLidarFromFile(lidarPoints, lidarFullFilename);

            // remove Lidar points based on distance properties
            // When driving on steep slope or over a bump, then minZ, ..., minR values have to be changed accordingly
            constexpr float minZ = -1.5, maxZ = -0.9;
            constexpr float minX = 2.0, maxX = 20.0;
            constexpr float maxY = 2.0, minR = 0.1; //(minR minimum reflectivity), focus on ego lane, they are in meters
            cropLidarPoints(lidarPoints, minX, maxX, maxY, minZ, maxZ, minR);
        
            // (dataBuffer.end() - 1)->lidarPoints = lidarPoints;
            (end(dataBuffer) - 1)->lidarPoints = lidarPoints; // Frame has following variables cameraImg, vectors -> keypoints, descriptors, kptMatches, lidarPoints, boundingBoxes, bbMatches.


            // std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;


            /* CLUSTER LIDAR POINT CLOUD */

            // associate Lidar points with camera-based ROI

            constexpr float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
            // clusterLidarWithROI((dataBuffer.end()-1)->boundingBoxes, (dataBuffer.end() - 1)->lidarPoints, shrinkFactor, P_rect_00, R_rect_00, RT); //shrinkFactor related to region of interest size
            clusterLidarWithROI((end(dataBuffer) - 1)->boundingBoxes,
                                (end(dataBuffer) - 1)->lidarPoints,
                                shrinkFactor, 
                                P_rect_00, 
                                R_rect_00, 
                                RT);
            // The output of clusterLidarWithROI will be that, each boundingBox in boundingBoxes will be associated with lidarPoints that lie within it. The lidarPoints in them can be seen by the lidarPoints property of boundingBox struct.
            // Visualize 3D objects
            bVis = false;
            if(bVis)
            {
            // show3DObjects((dataBuffer.end()-1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            show3DObjects((end(dataBuffer) - 1)->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), true);
            }
            bVis = false;

            // std::cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << std::endl;
            
            
            // REMOVE THIS LINE BEFORE PROCEEDING WITH THE FINAL PROJECT
            // continue; // skips directly to the next image without processing what comes beneath

            /* DETECT IMAGE KEYPOINTS */

            // convert current image to grayscale
            // We are converting the image to grayscale here to assist in keypoint detection. Yolo preferes colour images, for that reason, above we used colour images
            cv::Mat imgGray;
            // cv::cvtColor((dataBuffer.end()-1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY);
            cv::cvtColor((end(dataBuffer) - 1)->cameraImg, imgGray, cv::COLOR_BGR2GRAY); // Change the image stored at (end(dataBuffer) - 1)->cameraImg to Gray image


            // extract 2D keypoints from current image
            std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
            // std::string detectorType = "SHITOMASI";

            if (info.detectorType.compare("SHITOMASI") == 0) {
                data = detKeypointsShiTomasi(keypoints, imgGray, false); // data is dataType of DetectionData which has numKeyPoints, elapsedTime as variables that can be filled.  

            } else if (info.detectorType.compare("HARRIS") == 0) {
                data = detKeypointsHarris(keypoints, imgGray, false);

            } else {
                data = detKeypointsModern(keypoints, imgGray, info.detectorType, false);
            }
            
            // The std::array<int, 20> is set to 20 as ptsPerFrame has to store approximately 20 images data. 
            info.ptsPerFrame.at(imgIndex) = data.numKeyPoints; // info has detectorType, descriptorType, matcherType, selectorType, ptsPerFrame, matchedPts, ttcLidar, ttcCamera, enoughLidarOrCameraPointsDetected.

            // optional : limit number of keypoints (helpful for debugging and learning)
            constexpr bool bLimitKpts = false;
            if (bLimitKpts)
            {
                constexpr int maxKeypoints = 50;

                // if (detectorType.compare("SHITOMASI") == 0)
                // { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                //     keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                // }
                if (info.detectorType.compare("SHITOMASI") == 0) { // If the detectorType is SHITOMASI then only do this.
                    keypoints.erase(begin(keypoints) + maxKeypoints, end(keypoints)); // erase keypoints from maxKeypoints from begin(keypoints) to end(keypoints)
                }
                cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints); // For other cases do this. retainBest seems to be a method to filter keypoints. It takes into input keypoints and the maximum keypoints needed after filtering
                std::cout << " NOTE: Keypoints have been limited!" << std::endl;
            }

            // push keypoints and descriptor for current frame to end of data buffer
            // (dataBuffer.end() - 1)->keypoints = keypoints;
            (end(dataBuffer) - 1)->keypoints = keypoints;

            // std::cout << "#5 : DETECT KEYPOINTS done" << std::endl;


            /* EXTRACT KEYPOINT DESCRIPTORS */

            cv::Mat descriptors;
            // std::string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
            // descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);

            data = descKeypoints((end(dataBuffer) - 1)->keypoints,
                          (end(dataBuffer) - 1)->cameraImg,
                          descriptors,
                          info.descriptorType);

            // push descriptors for current frame to end of data buffer
            // (dataBuffer.end() - 1)->descriptors = descriptors;
            (end(dataBuffer) - 1)->descriptors = descriptors;


            // std::cout << "#6 : EXTRACT DESCRIPTORS done" << std::endl;


            if (dataBuffer.size() > 1) // wait until at least two images have been processed
            {

                /* MATCH KEYPOINT DESCRIPTORS */

                std::vector<cv::DMatch> matches;
                // std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                // std::string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
                // std::string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

                // matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                //                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                //                matches, descriptorType, matcherType, selectorType);
                const std::string descriptorFamily = (info.descriptorType.compare("SIFT") == 0) ? "DES_HOG" : "DES_BINARY" ;

                data = matchDescriptors((end(dataBuffer) - 2)->keypoints,
                                 (end(dataBuffer) - 1)->keypoints,
                                 (end(dataBuffer) - 2)->descriptors,
                                 (end(dataBuffer) - 1)->descriptors,
                                 matches,
                                 descriptorFamily,
                                 info.matcherType,
                                 info.selectorType);

                // store matches in current data frame
                // (dataBuffer.end() - 1)->kptMatches = matches;
                (end(dataBuffer) - 1)->kptMatches = matches;

                // std::cout << "#7 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;

                
                /* TRACK 3D OBJECT BOUNDING BOXES */

                //// STUDENT ASSIGNMENT
                //// TASK FP.1 -> match list of 3D objects (vector<BoundingBox>) between current and previous frame (implement ->matchBoundingBoxes)
                // map<int, int> bbBestMatches; // It is the result/return structure of matchBoundingBoxes
                std::map<int, int> bbBestMatches;
                // matchBoundingBoxes(matches, bbBestMatches, *(dataBuffer.end()-2), *(dataBuffer.end()-1)); // associate bounding boxes between current and previous frame using keypoint matches
                // This step is to discard BoundingBoxes which donot have any matched keypiont descriptor and also to find matchingBoundingBoxes pairs. 
                matchBoundingBoxes(matches,
                                   bbBestMatches, //bbBestMatches will be output of matchBoundingBoxes
                                   *(end(dataBuffer) - 2),
                                   *(end(dataBuffer) - 1));
                //// EOF STUDENT ASSIGNMENT

                // store matches in current data frame
                // (dataBuffer.end()-1)->bbMatches = bbBestMatches;
                // For the current frame, we write the map. The map is relative to the previous frame.
                (end(dataBuffer) - 1)->bbMatches = bbBestMatches;

                // std::cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << std::endl;


                /* COMPUTE TTC ON OBJECT IN FRONT */

                // loop over all BB match pairs. That is take each boundingBox matched pair and find TTC, TTL.
                // Matched bounding boxes pairs are stored in bbMatches variable of dataBuffer for current Frame.
                for (auto it1 = (dataBuffer.end() - 1)->bbMatches.begin(); it1 != (dataBuffer.end() - 1)->bbMatches.end(); ++it1)
                {
                    // find bounding boxes associates with current match
                    // it1 will have bbMatches first to last match partner.
                    BoundingBox *prevBB, *currBB;
                    // Looping over all the bounding boxes in current frame.
                    for (auto it2 = (dataBuffer.end() - 1)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 1)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->second == it2->boxID) // check wether current match partner corresponds to this BB. bbMatch is written as <prev, curr>, Thus second is used. 
                            currBB = &(*it2); // Here we get pointer to the bounding boxes of current frames
                    }
                    // Looping over all the bounding boxes in previous frame.
                    for (auto it2 = (dataBuffer.end() - 2)->boundingBoxes.begin(); it2 != (dataBuffer.end() - 2)->boundingBoxes.end(); ++it2)
                    {
                        if (it1->first == it2->boxID) // check wether current match partner corresponds to this BB. bbMatch is written as <prev, curr>, Thus second is used.
                            prevBB = &(*it2); // Here we get pointer to the bounding box of previous frames 
                    }
                            
                    // compute TTC for current match
                    // Check if the bounding boxes have lidar points associated with them in the current frame and the previous frame 
                    if( currBB->lidarPoints.size()>0 && prevBB->lidarPoints.size()>0 ) // only compute TTC if we have Lidar points
                    {
                        //// STUDENT ASSIGNMENT
                        //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
                        double ttcLidar; 
                        computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
                        
                        // Store Time to Collison calculated by Lidar for the current image set. We store it with respect of at(imgIndex).
                        info.ttcLidar.at(imgIndex) = ttcLidar;
                        //// EOF STUDENT ASSIGNMENT

                        //// STUDENT ASSIGNMENT
                        //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
                        //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
                        double ttcCamera;
                        // clusterKptMatchesWithROI(*currBB, 
                        //                         (dataBuffer.end() - 2)->keypoints, 
                        //                         (dataBuffer.end() - 1)->keypoints, 
                        //                         (dataBuffer.end() - 1)->kptMatches);                    
                        // computeTTCCamera((dataBuffer.end() - 2)->keypoints, 
                        //                 (dataBuffer.end() - 1)->keypoints, 
                        //                 currBB->kptMatches, 
                        //                 sensorFrameRate, 
                        //                 ttcCamera);

                        clusterKptMatchesWithROI(*currBB,
                                                 (end(dataBuffer) - 2)->keypoints,
                                                 (end(dataBuffer) - 1)->keypoints,
                                                 (end(dataBuffer) - 1)->kptMatches);

                        computeTTCCamera((end(dataBuffer) - 2)->keypoints,
                                         (end(dataBuffer) - 1)->keypoints,
                                         currBB->kptMatches,
                                         sensorFrameRate,
                                         ttcCamera);
                        
                        // Store Time to Collison calculated by Camera for the current image set. We store it with respect of at(imgIndex).
                        info.ttcCamera.at(imgIndex) = ttcCamera;
                        //// EOF STUDENT ASSIGNMENT
                        info.enoughLidarOrCameraPointsDetected.at(imgIndex) = true;


                        bVis = true;
                        if (bVis)
                        {
                            // cv::Mat visImg = (dataBuffer.end() - 1)->cameraImg.clone();
                            cv::Mat visImg = (end(dataBuffer) - 1)->cameraImg.clone();

                            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
                            cv::rectangle(visImg, 
                                        cv::Point(currBB->roi.x, currBB->roi.y), 
                                        cv::Point(currBB->roi.x + currBB->roi.width, 
                                        currBB->roi.y + currBB->roi.height), 
                                        cv::Scalar(0, 255, 0), 
                                        2);
                            
                            char str[200];
                            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
                            putText(visImg, 
                                    str, 
                                    cv::Point2f(80, 50), 
                                    cv::FONT_HERSHEY_PLAIN, 
                                    2, 
                                    cv::Scalar(0,0,255));

                            std::string windowName = "Final Results : TTC";
                            cv::namedWindow(windowName, 4);
                            cv::imshow(windowName, visImg);
                            std::cout << "Press key to continue to next frame" << std::endl;
                            // cv::waitKey(0);
                        }
                        bVis = false;

                    } else {
                        info.ttcCamera.at(imgIndex) = info.ttcLidar.at(imgIndex) = 0.0;
                        info.enoughLidarOrCameraPointsDetected.at(imgIndex) = false;
                    } 
                    // eof TTC computation
                } // eof loop over all BB matches            

            }
            std::cout << "IMAGE PROCESSED: " << imgFullFilename << std::endl;
        } // eof loop over all images
        std::cout << "======================\n" << std::endl;
    }
    createCSVOutputFile(timingInfo, numImages);
    return 0;
}
