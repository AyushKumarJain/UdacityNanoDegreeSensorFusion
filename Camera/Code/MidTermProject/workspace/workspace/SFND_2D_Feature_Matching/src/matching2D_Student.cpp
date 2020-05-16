#include <numeric>
#include "matching2D.hpp"

// using namespace std;



// MATCH DESCRIPTORS

// Find best matches for keypoints in two camera images based on several matching methods
// void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
//                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
DetectionData matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, const std::string descriptorFamily, const std::string matcherType, const std::string selectorType)
{
    // configure matcher
    double t;
    cv::Ptr<cv::DescriptorMatcher> matcher; //Pointer 

    if (matcherType.compare("MAT_BF") == 0)
    {
        constexpr bool crossCheck = false; // Usually crossCheck is used for Brute Force Matching
        // const int normType = cv::NORM_HAMMING;
        const int normType{ ((descriptorFamily.compare("DES_BINARY") == 0) ? cv::NORM_HAMMING : cv::NORM_L2) }; // Depending on descriptor type (Binary or Intensity gradients), Hamming or L2 is used.
        matcher = cv::BFMatcher::create(normType, crossCheck);
        std::cout << "BF matching cross-check = " << crossCheck << std::endl;
    }else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // You have to take care of datatype when we have to enter data to FLANN
        if (descRef.type() != CV_32F) 
            descRef.convertTo(descRef, CV_32F); 

        if (descSource.type() != CV_32F) 
            descSource.convertTo(descSource, CV_32F); 

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::cout << "FLANN matching" << std::endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
    
        t = static_cast<double>(cv::getTickCount());
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((static_cast<double>(cv::getTickCount())) - t) / cv::getTickFrequency();
    
    }else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        std::vector<std::vector<cv::DMatch>> knnMatches;
        t = static_cast<double>(cv::getTickCount());
        matcher->knnMatch(descSource, descRef, knnMatches, 2);
        constexpr float threshold{ 0.8 };

        for (auto iterator = std::begin(knnMatches); iterator != std::end(knnMatches); iterator++) 
        {
            if ((*iterator).at(0).distance < (threshold * (*iterator).at(1).distance)) 
            {
                matches.push_back((*iterator).at(0));
            }
        }

        t = ((static_cast<double>(cv::getTickCount())) - t) / cv::getTickFrequency();  
    }

    return DetectionData{ static_cast<int>(matches.size()), secondsToMilliseconds(t) };
}






// GENERATE DESCRIPTORS

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
//void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
DetectionData descKeypoints(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, const std::string descriptorType)
{
    // select appropriate descriptor
    // BRISK, ORB, FREAK, AKAZE, SIFT, BRIEF
    cv::Ptr<cv::DescriptorExtractor> extractor; // It is a template pointer, as we are not going to change cv::DescriptorExtractor type data structure.
    if (descriptorType.compare("BRISK") == 0)
    {
        constexpr int threshold = 30;        // FAST/AGAST detection threshold score.
        constexpr int octaves = 3;           // detection octaves (use 0 to do single scale)
        constexpr float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (descriptorType.compare("ORB") == 0) {
        extractor = cv::ORB::create();

    } else if (descriptorType.compare("FREAK") == 0) {
        extractor = cv::xfeatures2d::FREAK::create();

    } else if (descriptorType.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create();

    } else if (descriptorType.compare("SIFT") == 0) {
        extractor = cv::xfeatures2d::SIFT::create();

    } else if (descriptorType.compare("BRIEF") == 0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

    // perform feature description
    double t = static_cast<double>(cv::getTickCount()); // (double)cv::getTickCount(); This is the expression when we will "using namespace std"
    extractor->compute(img, keypoints, descriptors);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << std::endl;

    return DetectionData{ static_cast<int>(keypoints.size()), secondsToMilliseconds(t) };
}








// DETECT KEYPOINTS
// OLD 
// Detect keypoints in image using the traditional Shi-Thomasi detector
// void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
DetectionData detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const bool bVis)
{
    // compute detector parameters based on image size
    constexpr int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    constexpr double maxOverlap = 0.0; // max. permissible overlap between two features in %
    constexpr double minDistance = (1.0 - maxOverlap) * blockSize;
    const int maxCorners = img.rows * img.cols / std::max(1.0, minDistance); // max. num. of keypoints

    constexpr double qualityLevel = 0.01; // minimal accepted quality of image corners
    constexpr double k = 0.04;

    // Apply corner detection
    double t = static_cast<double>(cv::getTickCount());
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = std::begin(corners); it != std::end(corners); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();
    std::cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    // visualize results
    if (bVis)
    {
        const cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return DetectionData{ static_cast<int>(keypoints.size()), secondsToMilliseconds(t) };
}
// Student Code Start

DetectionData detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const bool bVis) 
{
    constexpr int blockSize{ 2 };
    constexpr int apertureSize{ 3 };
    constexpr int minResponse{ 100 };
    constexpr double k{ 0.04 };
    constexpr double overlapThreshold{ 0.0 };
    constexpr int scaledApertureSize{ apertureSize * 2 };

    double t = static_cast<double>(cv::getTickCount());

    cv::Mat dst, dstNorm, dstNormScaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dstNorm, dstNormScaled);

    bool foundOverlap{ false };

    for (int i{ 0 }; i < dstNorm.rows; i++) {
        for (int j{ 0 }; j < dstNorm.cols; j++) {

            const int response{ static_cast<int>(dstNorm.at<float>(i, j)) };

            if (response > minResponse) {
                cv::KeyPoint point;
                point.pt = cv::Point2f(i, j);
                point.size = scaledApertureSize;
                point.response = response;
                point.class_id = 0;

                foundOverlap = false;

                for (auto iterator{ std::begin(keypoints) }; iterator != std::end(keypoints); iterator++) {
                    if (cv::KeyPoint::overlap(point, (*iterator)) > overlapThreshold) {
                        foundOverlap = true;

                        if (point.response > (*iterator).response) {
                            *iterator = point;
                            break;
                        }
                    }
                }

                if (!foundOverlap) { keypoints.push_back(point); }
            }
        }
    }

    t = ((static_cast<double>(cv::getTickCount())) - t) / cv::getTickFrequency();

    if (bVis) {
        const cv::Mat visImage{ img.clone() };
        constexpr char windowName[]{ "Harris Detector Results" };

        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(windowName, 6);

        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return DetectionData{ static_cast<int>(keypoints.size()), secondsToMilliseconds(t) };
}

// NEW
DetectionData detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const std::string detectorType, const bool bVis) 
{
    double t;
    cv::Ptr<cv::FeatureDetector> detector;
    // Note that below FREAK has been removed from the keypoint detectors, as there are some bugs in the OpenCV implementation.
    if (detectorType.compare("FAST") == 0) {
        // TYPE_9_16, TYPE_7_12, TYPE_5_8
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
        detector = cv::FastFeatureDetector::create(30, true, type);

    } else if (detectorType.compare("BRISK") == 0) {
        detector = cv::BRISK::create();

    } else if (detectorType.compare("ORB") == 0) {
        detector = cv::ORB::create();

    } else if (detectorType.compare("AKAZE") == 0) {
        detector = cv::AKAZE::create();

    } else if (detectorType.compare("SIFT") == 0) {
        detector = cv::xfeatures2d::SIFT::create();

    }

    t = static_cast<double>(cv::getTickCount());
    detector->detect(img, keypoints);
    t = (static_cast<double>(cv::getTickCount()) - t) / cv::getTickFrequency();

    if (bVis) {
        const std::string windowName(detectorType + " detection results.");
        cv::Mat visImage{ img.clone() };
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::namedWindow(windowName, 5);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return DetectionData{ static_cast<int>(keypoints.size()), secondsToMilliseconds(t) };
}

inline const double secondsToMilliseconds(const double seconds) 
{
    return ((1000 * seconds) / 1.0);
}
// Student Code End