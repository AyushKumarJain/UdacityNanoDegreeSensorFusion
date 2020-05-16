// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

// The inputs to the FilterCloud are input cloud, voxel grid size, and min/max points representing our region of interest.
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    const auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    // Create the filtering object
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);

    typename pcl::PointCloud<PointT>::Ptr filteredVoxelCloud (new pcl::PointCloud<PointT>);
    voxelGrid.filter(*filteredVoxelCloud);

    pcl::CropBox<PointT> cropBox(true);
    cropBox.setInputCloud(filteredVoxelCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);

    typename pcl::PointCloud<PointT>::Ptr filteredCroppedCloud (new pcl::PointCloud<PointT>);
    cropBox.filter(*filteredCroppedCloud);

    pcl::CropBox<PointT> vehicleRoof(true);
    vehicleRoof.setInputCloud(filteredCroppedCloud);
    vehicleRoof.setMin(Eigen::Vector4f (-1.5, -1.7, -1.1, 1.0));
    vehicleRoof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1.0)); //2.6, 1.7, -4.1, 1.0 

    std::vector<int> inlierIndices;
    vehicleRoof.filter(inlierIndices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : inlierIndices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extractedIndices;
    extractedIndices.setInputCloud(filteredCroppedCloud);
    extractedIndices.setIndices(inliers);
    extractedIndices.setNegative(true);
    extractedIndices.filter(*filteredCroppedCloud);

    const auto endTime = std::chrono::steady_clock::now();
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return filteredCroppedCloud;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{  
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  // Here we have created two new point cloud pointers on the heap, one for obstacles (non-plane points), and one for road (plane points)
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT> ());
    
    // The inliers are added to the plane cloud by looping over the inlier indices and pushing the corresponding inlier point into the plane cloud’s point vector.
    for(int index : inliers -> indices)
        roadCloud -> points.push_back(cloud->points[index]);

    // Create the filtering object
    // Below created extract object will subtract the plane cloud from the input cloud.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);

    extract.setNegative (false);
    extract.filter (*roadCloud);

    extract.setNegative (true);
    extract.filter (*obstacleCloud); //* is used for dereferencing
    
    // The std::pair below is returned with the newly created obstacle and plane clouds.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}

// SegmentPlane does, Planar Segmentation which uses the RANSAC (random sample consensus) algorithm.
// A template parameter PointT is used as a variable to represent any type of point cloud.
// The function accepts a point cloud, max iterations, and distance tolerance as arguments.
// SegmentPlane function will return a std::pair holding point cloud pointer types
// The pair object is needed to hold segmented results for the obstacle point cloud and the road point cloud.
// It will help to visualize both point clouds in the pcl viewer.
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    const auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg; // We use PointT type here as, it will help us process any type of points (eg. with intensity values also)
    // Creating inliers object using pcl::PointIndices, inliers will have indices for the fitted plane
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; // Objects created on heap
    // coefficents object below is made to define a plane (its parameters) that we want to segment
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients} ; // Objects created on heap
    
    seg.setOptimizeCoefficients (true);
    seg.setMaxIterations (maxIterations); //RANSAC needs it
    seg.setDistanceThreshold (distanceThreshold); //This too
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC); //RANSAC is the algorithm
    seg.setInputCloud (cloud); // Segment the largest planar component from the input cloud
    seg.segment (*inliers, *coefficients); //* is used as we are dereferencing here, as inliers is initialised as a pointer. We want actual inlier values. seg.segment will generate for us inliers.
    
    if (inliers->indices.size () == 0) // If we donot find any model that fit this data
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    
    const auto endTime = std::chrono::steady_clock::now();
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}

// Old Clustering Function with build in KdTree

/* template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize)
{
    // Time clustering process
    const auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Here new is used to creat a Lidar pointer object on the heap.
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); 
    tree->setInputCloud (cloud); // The KdTree object is called tree, The cloud is set as InputCloud to KdTree

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // Hyperparameter (Comes from arguments in the function header)
    ec.setMinClusterSize (minSize); // Hyperparameter
    ec.setMaxClusterSize (maxSize); // Hyperparameter
    ec.setSearchMethod (tree); 
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);
    
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (pcl::PointIndices getIndices: clusterIndices) // pcl::PointIndices is variable type, getIndices is variable 
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>); //We want to tell the compiler it is a type, while using that template   
        // The cloudCluster PointCloud is created to store a PointCloud iteratively, depending on the PointIndices getIndices from clusterIndices
        
        for (int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]); // The points in the cloud PointCloud are push_back in to cloudCluster PointCloud, 
            
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }


    const auto endTime = std::chrono::steady_clock::now();
    const auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
 */

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud); //*cloud gives value of the pointer, cloud returns memory address as pcl::PointCloud<PointT>::Ptr helps generates pointer.
    std::cerr << "Saved " << cloud->points.size () << " data points to " + file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); // Initialised on heap

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
        PCL_ERROR ("Couldn't read file \n");
    
    std::cerr << "Loaded " << cloud->points.size () << " data points from " + file << std::endl; // That is why we use ->

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


// New additions Start
// RANSAC for Plane
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, const float distanceTol) 
{

    std::unordered_set<int> inliersResult;
    // TODO: Fill in this function
    srand(time(NULL));

    while (maxIterations--) 
    {
        // Randomly pick three points
        std::unordered_set<int> inliers; //Can hold my best inliers result.
        while (inliers.size() < 3)
			inliers.insert(rand()%(cloud -> points.size())); //rand is a function, gives value between 0 and pointsize of clouds

        // const float x1, y1, z1, x2, y2, z2, x3, y3, z3 ;   

        auto itr = inliers.begin(); // auto helps us set pointer itr, that points at the very beginning of the inliers
		const float x1 = cloud -> points[*itr].x; // The * is for de referencing the pointer (we dereference to show value at the pointer) and .x will help us show the value.
		const float y1 = cloud -> points[*itr].y;
        const float z1 = cloud -> points[*itr].z;
		itr++;
		const float x2 = cloud -> points[*itr].x;
		const float y2 = cloud -> points[*itr].y;
        const float z2 = cloud -> points[*itr].z;
        itr++;
		const float x3 = cloud -> points[*itr].x;
		const float y3 = cloud -> points[*itr].y;
        const float z3 = cloud -> points[*itr].z;

		const float A = static_cast<float>((y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1));
		const float B = static_cast<float>((z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1));
		const float C = static_cast<float>((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
        const float D = -static_cast<float>(A * x1 + B * y1 + C * z1);
        const float euclDist = std::sqrt(A*A + B*B + C*C);
 
        for (int index = 0; index < cloud -> points.size(); index++) 
        {
            if(inliers.count(index)>0) //.count() helps check if the index is part of the inliers or not. If inlier contains the element, it will return 1. Thus, 
				continue; // continue command will not allow the following code to run, it will go to next iteration
			
            const float plane = std::fabs(A * cloud->points[index].x + B * cloud->points[index].y + C * cloud->points[index].z + D) ;
              
            if ((plane/euclDist) <= distanceTol) 
                inliers.insert(index); 
        }

        if (inliers.size() > inliersResult.size()) 
            inliersResult = inliers; 
    }
    typename pcl::PointCloud<PointT>::Ptr cloudInliers {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers {new pcl::PointCloud<PointT>()};

    if (!inliersResult.empty()) 
    {
        for (int index = 0; index < cloud -> points.size(); index++)
        {
            const PointT point {cloud -> points[index]};
            if (inliersResult.count(index))
            {   
                cloudInliers -> points.push_back(point);
            } 
            else 
            {   
                cloudOutliers -> points.push_back(point);
            }
        }
    }

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(cloudOutliers, cloudInliers);

}  

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(const int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, const float tolerance) 
{
    processed.at(index) = true;
    cluster.push_back(index);

    const std::vector<int> nearbyPoints{ tree->search(points.at(index), tolerance) };

    for (int nearbyIndex : nearbyPoints) 
    {
        if (!processed.at(nearbyIndex)) 
            proximity(nearbyIndex, points, cluster, processed, tree, tolerance);
        
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol) // points have same location points as points in clustering algorithm, but they will remain constant in both euclidean algorithm and proximity
{
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    for (int i{ 0 }; i < points.size(); i++) 
    {
        if (!processed.at(i)) 
        {
            std::vector<int> cluster;
            proximity(i, points, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }

    return clusters;
}

// Clustering with KdTree implmented in kdtree.h
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float clusterTolerance, const int minSize, const int maxSize) 
{
    const auto startTime{ std::chrono::steady_clock::now() };

    int i {0};
    KdTree *kdTree {new KdTree()};

    std::vector<std::vector<float>> points;

    for (auto point : cloud->points) 
    {
        const std::vector<float> p {point.x, point.y, point.z};
        kdTree->insert(p, i++);
        points.push_back(p);
    }

    const std::vector<std::vector<int>> listOfIndices {euclideanCluster(points, kdTree, clusterTolerance)}; //points have simple points, while kdTree has kdTree

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (auto indices : listOfIndices) 
    {
        if (indices.size() < minSize || indices.size() > maxSize) 
            continue; 

        typename pcl::PointCloud<PointT>::Ptr cluster{ new pcl::PointCloud<PointT> };

        for (auto index : indices) 
            cluster->points.push_back(cloud->points[index]); 

        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    const auto endTime {std::chrono::steady_clock::now()};
    const auto elapsedTime {std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime)};

    std::cout << "clustering took " << elapsedTime.count();
    std::cout << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// New additions End