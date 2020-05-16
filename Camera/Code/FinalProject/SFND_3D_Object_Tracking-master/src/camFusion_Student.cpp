
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

// using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, const std::vector<LidarPoint> &lidarPoints, const float shrinkFactor, const cv::Mat &P_rect_xx, const cv::Mat &R_rect_xx, const cv::Mat &RT) 
{   
    using std::end;
    using std::begin;
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    const float oneMinusShrinkFactor = 1 - shrinkFactor ;
    const cv::Mat partialProjection = P_rect_xx * R_rect_xx * RT ; // no need to recompute this every single iteration
       

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;
 
        // project Lidar point into camera
        //Y = P_rect_xx * R_rect_xx * RT * X;
        Y = partialProjection * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        // pointers to all bounding boxes which enclose the current Lidar point
        std::vector<std::vector<BoundingBox>::iterator> enclosingBoxes; 
        // Iterating over all the bounding boxes to check in which shrinked bounding box does the lidar point lies ?
        for (std::vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            // smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            // smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);
            smallerBox.width = (*it2).roi.width * oneMinusShrinkFactor;
            smallerBox.height = (*it2).roi.height * oneMinusShrinkFactor;

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2); // Add all the shrinked BoundingBox that enclose the lidar point into enclosingBoxes
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1); // Taking the first boundingBox of the enclosingBoxes. If there is only one boundingBox(it2) in which lidarPoint (it1) resides then, push_back that lidar point into lidarPoints of the first boundingBox of the enclosingBoxes. 
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, const cv::Size worldSize, const cv::Size imageSize, const bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    std::string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        // cv::waitKey(0); // wait for key to be pressed
    }
}


/* // associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // ...
}

// Use keypoint matches for previous and current frame, that will be outer loop. Check bounding boxes which enclose the keypoint matches in both previous and current frame. There boxId will be needed to the stored in multimap. In the end count them. Associate bounding boxes with highest number of occurences.
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
} */

// Student Code Start



void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

	const int columns = static_cast<int>(prevFrame.boundingBoxes.size()); // Finding size of bounding box of previous frame and storing in columns
	const int rows = static_cast<int>(currFrame.boundingBoxes.size()); // Finding the size of bounding box of current frame and storing in rows
    
    // Example code is shown below
    // Initialisation of listOfMatches has all zeros at all the places
    std::vector<std::vector<int>> listOfMatches(columns, std::vector<int>(rows, 0)); // Its like table where we can know which boundingBoxes of prevFrame related to which boundingBoxes of currFrame

    int previousFrameMatchingBoundingBoxId = -1;
    int currentFrameMatchingBoundingBoxId = -1 ;
    // This loop, loops over all the matches
    // This for loop will give as output listOfMatches. i.e. all the BoxId's in prevFrame and currFrame that match. It will take care that there are only one to one match in both direction.
	for(auto match : matches)
    {
		if(!matchPointToBoundingBox(prevFrame, match.queryIdx, previousFrameMatchingBoundingBoxId)) // The if is asking to find if the current matched descriptor (stored in match) of the prevFrame matches to some bounding box or not. If it doesnot then just continue, donot execute the next step after if
            continue; 
		if(!matchPointToBoundingBox(currFrame, match.trainIdx, currentFrameMatchingBoundingBoxId)) // This if is asking to find if the current matched descriptor (stored in match) of the currFrame matches to some bounding box or not. If it doesnot just continue, donot execute the next step after if
            continue; 
        ++listOfMatches.at(previousFrameMatchingBoundingBoxId).at(currentFrameMatchingBoundingBoxId); // When both values will be obtained then only listOfMatches will increase its value
	}
    // This loop loops over all the boundingBoxes of prevFrame.
    // In the previous for loop, as a result, we will get listOfMatches std::vector<std::vector<int>> type, filled with number of matchpoints corresponding to all the combination of boundingBox.
    for (int columnIndex = 0; columnIndex < prevFrame.boundingBoxes.size(); columnIndex++) 
    {
        const int rowIndex = getMaximumIndex(listOfMatches.at(columnIndex)); // For a columnIndex, MaximumIndex is sougth
        // We want only 1 rowIndex corresponding to a columnIndex.
        if (listOfMatches.at(columnIndex).at(rowIndex) == 0) // If there are no matchpoints in a boundingBox combination, leave that boundingBox combination
            continue; 
        bbBestMatches[prevFrame.boundingBoxes.at(columnIndex).boxID] = currFrame.boundingBoxes.at(rowIndex).boxID; // Mapping variables. 
    }
}

inline const bool matchPointToBoundingBox(const DataFrame &frame, const int index, int &matchingBoxId) 
{
	int matchCount = 0;
    // Index for prevFrame stores the descriptor index i.e. queryIdx, Similarly for currFrame, it stores the trainIdx. Both this index tell about descriptor position in the Frame  
    const cv::Point2f point(frame.keypoints.at(index).pt);
    
    // Iterating over all the bounding boxes of the frame (the frame as input to the function).
    for (int currentBoundingBoxIndex = 0; currentBoundingBoxIndex < frame.boundingBoxes.size(); currentBoundingBoxIndex++) 
    {
        // The if condition checks if the frame's boundingBoxes at the for loop's curretBoundingBoxIndex has the keypoint describing the descriptor. If it doesnot it continues and checks the next bounding box. But if does has then number of detectors/descriptors are counted to be present.
        if (!frame.boundingBoxes.at(currentBoundingBoxIndex).roi.contains(point)) 
            continue; 
        // If the point corresponding to index, which in turn corresponds to match, is found in the currentBoundingBox, increase the matchCount. But we can enter inside the if() only when matchCount is more than 1.
        // Thus for the first point, we cannot go inside the matchCount. Once the matchCount is more than 1, i.e. The point/index/match is within two boundingBoxes, then set matchBoxID = -1 (i.e. we donot want to consider it).
        
        if (++matchCount > 1) 
        {
            matchingBoxId = -1;
            return false;
        }
        // If not, then assign currentBoundingBoxIndex to matchingBoxId.
        matchingBoxId = currentBoundingBoxIndex;
    }
    
    return static_cast<bool>(matchCount);
    // Thus the output of this function is 1. The matchingBoxId to the point/index/match, 2. matchCount, i.e. a keypoint/index/descriptors match with the boundBoxes 
    // Ideally matchCount should not exceed 1, matchingBoxId should have a positive number rather than -1, 
	
}

inline const int getMaximumIndex(const std::vector<int> &inputData)
{
    using std::begin;
    using std::end;

	return std::distance(begin(inputData), std::max_element(begin(inputData), end(inputData)));
}





void computeTTCLidar(const std::vector<LidarPoint> &lidarPointsPrev, const std::vector<LidarPoint> &lidarPointsCurr, const double frameRate, double &TTC) 
{

    constexpr int DOES_NOT_MATTER = 0 ;
    const double dT = 1 / frameRate ;

    double averageXPrev = 0.0;
    double averageXCurr = 0.0;
    double minXCurr = 1e9;

    for (auto point : lidarPointsPrev) 
        averageXPrev += point.x;
    for (auto point : lidarPointsCurr) 
        averageXCurr += point.x; 

    if (!lidarPointsPrev.empty()) 
        averageXPrev /= static_cast<int>(lidarPointsPrev.size()); 
    if (!lidarPointsCurr.empty()) 
        averageXCurr /= static_cast<int>(lidarPointsCurr.size());  

    for (auto point : lidarPointsCurr) 
    {
        const double threshold = 0.75 * averageXCurr ; // As we want to handle erroneaus lidar points x direction measurements (sometimes some points are measured very near to lidar too, that we donot want to consider), we calculate a threshold. We want points that are more than average.

        if (point.x > 0.0 && point.x > threshold) 
            minXCurr = point.x; 
    }

    TTC = (minXCurr * dT) / (averageXPrev - averageXCurr);
}




// associate a given bounding box with the keypoints it contains
// The function adds the keypoint correspondences to the "kptMatches" property of the respective bounding boxes. 
// This function works with 1 boundingBox each time. For each boundingBox it needs to find kptMatches
void clusterKptMatchesWithROI(BoundingBox &boundingBox, const std::vector<cv::KeyPoint> &kptsPrev, const std::vector<cv::KeyPoint> &kptsCurr, const std::vector<cv::DMatch> &kptMatches) 
{
    using std::begin;
    using std::end;

	std::vector<double> distance;

    for (auto point : kptMatches) 
    {
		const auto &kptCurr = kptsCurr.at(point.trainIdx);

		if (!boundingBox.roi.contains(kptCurr.pt)) 
            continue; 

        const auto &kptPrev = kptsPrev.at(point.queryIdx);
        distance.push_back(cv::norm(kptCurr.pt - kptPrev.pt));
	}

	const int distanceNum = static_cast<int>(distance.size());

	const double distanceMean = std::accumulate(begin(distance), end(distance), 0.0) / distanceNum;
    const double scaledDistanceMean = distanceMean * 1.3;

    for (auto point : kptMatches) 
    {
		const auto &kptCurr = kptsCurr.at(point.trainIdx);

		if (!boundingBox.roi.contains(kptCurr.pt)) 
            continue; 

        const int kptPrevIdx = point.queryIdx;
        const auto &kptPrev = kptsPrev.at(kptPrevIdx);

        if (cv::norm(kptCurr.pt - kptPrev.pt) < scaledDistanceMean) 
        {
            boundingBox.keypoints.push_back(kptCurr);
            boundingBox.kptMatches.push_back(point);
        }
	}
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
// TTC 
void computeTTCCamera(const std::vector<cv::KeyPoint> &kptsPrev, const std::vector<cv::KeyPoint> &kptsCurr, const std::vector<cv::DMatch> kptMatches, const double frameRate, double &TTC, cv::Mat *visImg) // *visImg is a optional visualisation image
{
    using std::end;
    using std::begin;

    constexpr int DOES_NOT_MATTER = 0;

    std::vector<double> distRatios;
    constexpr double minDist = 100.0;
    // Looping over all the keypoint matches.
    for (auto outerKptMatch = begin(kptMatches); outerKptMatch != end(kptMatches) - 1; ++outerKptMatch) 
    {
        const cv::KeyPoint kpOuterCurr = kptsCurr.at(outerKptMatch->trainIdx);
        const cv::KeyPoint kpOuterPrev = kptsPrev.at(outerKptMatch->queryIdx);

        for (auto innerKptMatch = begin(kptMatches) + 1; innerKptMatch != end(kptMatches); ++innerKptMatch) 
        {
            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(innerKptMatch->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(innerKptMatch->queryIdx);

            // compute distances and distance ratios
            const double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            const double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist) 
            {
                const double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.empty()) 
    {
        TTC = NAN;
        return;
    }

    std::sort(begin(distRatios), end(distRatios));
    // Finding the median index. 
    const long medIndex = static_cast<long>(std::floor(distRatios.size() / 2.0));

    // compute median dist. ratio to remove outlier influence
    double medDistRatio;
    // Depending on the size of the number of distRations found in the above 2 for loop. The distRatios is computed below.
    if (distRatios.size() % 2 == 0) 
    {
        medDistRatio = (distRatios.at(medIndex - 1) + distRatios.at(medIndex)) / 2.0;
    } else {
        medDistRatio = distRatios.at(medIndex);
    }

    const double dT = 1 / frameRate;

    TTC = -dT / (1 - medDistRatio);
}



// Student Code End


/* #include <iostream>
#include <vector>

using namespace std;

int main(){ 
    int n = 10; 
    int m = 5; 
  
    // Create a vector containing n 
    //vectors of size m.  
    vector<vector<int> > vec( n , vector<int> (m, 0));  
  
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j < m; j++){ 
            vec[i][j] = j + i + 1; 
        } 
    } 
  
    for (int i = 0; i < n; i++) { 
        for (int j = 0; j < m; j++){ 
            cout<< vec[i][j]<< " "; 
        } 
        cout<< "\n"; 
    } 
    
    ++vec.at(0).at(0);
    
        for (int i = 0; i < n; i++) { 
        for (int j = 0; j < m; j++){ 
            cout<< vec[i][j]<< " "; 
        } 
        cout<< "\n"; 
    } 
    
  
return 0; 
}  */