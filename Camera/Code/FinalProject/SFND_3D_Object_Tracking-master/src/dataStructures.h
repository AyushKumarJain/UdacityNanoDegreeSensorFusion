
#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <map>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct BoundingBox { // bounding box around a classified object (contains both 2D and 3D data)
    
    int boxID; // unique identifier for this bounding box
    int trackID; // unique identifier for the track to which this bounding box belongs
    
    cv::Rect roi; // 2D region-of-interest in image coordinates
    int classID; // ID based on class file provided to YOLO framework
    double confidence; // classification trust

    std::vector<LidarPoint> lidarPoints; // Lidar 3D points which project into 2D image roi
    std::vector<cv::KeyPoint> keypoints; // keypoints enclosed by 2D roi
    std::vector<cv::DMatch> kptMatches; // keypoint matches enclosed by 2D roi
};

struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    // All the cv::KeyPoint are stored in keypoints
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    // All the descritors of cv::KeyPoint are stored in rows of cv::Mat
    cv::Mat descriptors; // keypoint descriptors
    // All the cv::DMatch matched cv::KeyPoint are stored in kptMatches
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
    // All the lidar points of type LidarPoint stored in lidarPoints
    std::vector<LidarPoint> lidarPoints;
    // All the BoundingBox stored in boundingBoxes
    std::vector<BoundingBox> boundingBoxes; // ROI around detected objects in 2D image coordinates
    std::map<int,int> bbMatches; // bounding box matches between previous and current frame
};

// Student Code Start
struct DetectionData {
    int numKeyPoints;
    double elapsedTime;

    // constructors
    DetectionData() : numKeyPoints(0), elapsedTime(0.0) {} // Normally the dataStructure DetectionData has 0 numKeyPoints and 0.0 elapsedTime value

    DetectionData(int points, double time) : numKeyPoints(points), elapsedTime(time) {}
};

struct TimingInfo {
    const std::string detectorType;
    const std::string descriptorType;
    const std::string matcherType;
    const std::string selectorType;

    std::array<int, 20> ptsPerFrame; // What does mean by 20 here ?
    std::array<int, 20> matchedPts; 

    std::array<double, 20> ttcLidar; // What does mean by 20 here ?
    std::array<double, 20> ttcCamera;

    std::array<bool, 20> enoughLidarOrCameraPointsDetected;

    // constructors
    TimingInfo() {}

    TimingInfo(const std::string detType, const std::string descType, const std::string matchType, const std::string selType)
        : detectorType(detType), descriptorType(descType), matcherType(matchType), selectorType(selType) {}
};

// Student Code End

#endif /* dataStructures_h */
