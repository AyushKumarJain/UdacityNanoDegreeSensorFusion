/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
// lidar.h header file has lidar object.
// render.h header file has rendering object.  
//It is needed to include them at the top of the environment.cpp file.
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

//pcl::visualization::PCLVisualizer::Ptr& viewer used by initCamera to visualise the objects on the screen
//viewer is usually passed as reference (Ptr&). Referencing helps. It donot want anything to be returned.
//Highway objects
std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer); //renderHighway seems to be some method/function of viewer
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

// simpleHighway function also take reference argument of PCL visualizer viewer
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // renderScene set to true will help view the point cloud without obstacles
    bool renderScene = false; // true
    bool render_obst = false;
    bool render_plane = false;
    bool render_clusters = true;
    bool render_box = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // Here we instantiate a pointer to a Lidar object by using Lidar*
    // The created object name is lidar (starts with small l), it is Lidar object's constructor
    // Here new is used to creat a Lidar pointer object on the heap.
    // The constructor of Lidar object takes, cars and slope of the ground, they are necessary to model ray collision. 
    // The Lidar object (lidar) is going to be holding point cloud data which could be very large. 
    // By instatinating on the heap, we have more memory to work with than the 2MB on the stack. 
    // However, it takes longer to look up objects on the heap, while stack lookup is very fast.
    Lidar* lidar = new Lidar(cars,0); // 0 indicate we create it with zero slope 
    // The PointCLoud syntax has ContainerName<ObjectName>. The Ptr type from PointCloud indicates that the object is actually a pointer - a 32 bit integer that contains the memory address of your point cloud object. 
    // Calling lidar scan function to render the lidar rays and storing the results in a PointCloud pointer object, pcl::PointXYZ is the type of point in point cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar -> scan(); 
    // renderRays function is used to render rays.
    //renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud function is used to render just the pointcloud.
    //renderPointCloud(viewer, inputCloud, "inputCloud"); // "inputCloud" is used to show name in the viewer
    
    // TODO:: Create point processor
    
    // Segmentation Rendering
    // The ProcessPointClouds has pcl::PointXYZ type point cloud, or  ContainerName<ObjectName>
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; //OR // It creates object on the stack
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    
    // The std::pair with the newly created obstacle and plane clouds.
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2); // . or ->, choice depends on instantiating from stack or heap 
    if(render_obst)
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    if(render_plane)
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    
    //Cluster Rendering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30); // Hyperparameters to be set with experience, They are min and max points in cluster and the avg. radius of cluster.

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; // red, yellow, blue

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {   
        if(render_clusters)
        {
            std::cout << "cluster size ";
            pointProcessor.numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
        
        }
        
        if (render_box)
        {
            Box box = pointProcessor.BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
        ++clusterId;
    }

}


/* // For single pcd files
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
    // Single pcd file rendering
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>(); //heap and stack are two different things, they change initialisation 
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    //Single pcd file filtered than rendering
    // Experiment with the ? values and find what works best, this part doesnot work best. 
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-30.0, -4, -3.0, 1), Eigen::Vector4f (30.0, 4, 3.0, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");
    constexpr float X{30.0}, Y{5}, Z{2};
    inputCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-X/2, -Y, -Z, 1), Eigen::Vector4f (X, Y, Z/2, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(inputCloud, 25, 0.3);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));   

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.53, 10, 500); // Hyperparameters to be set with experience, They are min and max points in cluster and the avg. radius of cluster.

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {   
        std::cout << "cluster size ";
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

    //renderPointCloud(viewer,inputCloud,"inputCloud");
}    */ 

/* // For mutiple pcd file, i.e. for stream
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{ 
    //constexpr float X{30.0}, Y{5}, Z{2};
    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-15.0, -5.0, -2.0, 1.0), Eigen::Vector4f (30.0, 5.0, 1.0, 1.0));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 25, 0.3);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.53, 10, 500); // Hyperparameters to be set with experience, They are min and max points in cluster and the avg. radius of cluster.

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {   
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }

    //renderPointCloud(viewer,inputCloud,"inputCloud");
} */

// Student code start
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) // const and &inputCloud are used for passing by reference, to not change value of input cloud. But viewer is also passed by reference but without const, i.e. its values are prone to change.
{

    ProcessPointClouds<pcl::PointXYZI> *pointProcessor{ new ProcessPointClouds<pcl::PointXYZI>() };
    constexpr float X{ 30.0 }, Y{ 6.5 }, Z{ 2.5 };

    const pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud{ pointProcessor->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f(-(X / 2), -Y, -Z, 1), Eigen::Vector4f(X, Y, Z, 1)) };
    renderPointCloud(viewer, filteredCloud, "filteredCloud");
    const std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud{pointProcessor->RansacPlane(filteredCloud, 20, 0.2)};
        //     pointProcessor->SegmentPlane(filteredCloud, 100, 0.2)
        
    if (segmentCloud.first->empty() || segmentCloud.second->empty()) { return; }

    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "Plane Cloud", Color(0, 1, 0));
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters{ pointProcessor->Clustering(
        segmentCloud.first, 0.35, 30, 1775) };

    if (cloudClusters.empty()) { return; }

    int clusterId{ 0 }, colorIndex{ 0 };
    const std::vector<Color> colors{ Color(1, 0, 1), Color(0, 1, 1), Color(1, 1, 0) };
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) 
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(colorIndex));
        Box box(pointProcessor->BoundingBox(cluster));
        renderBox(viewer, box, clusterId);
        ++clusterId;
        ++colorIndex;
        colorIndex %= colors.size();
    }
}
// Student code end


//initCamera method/function helps in set up different viewing angles in output window
//pcl::visualization::PCLVisualizer::Ptr& viewer used by initCamera to visualise the objects on the screen
//viewer is usually passed as reference (Ptr&). Referencing helps. It donot want anything to be returned.
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);
    
    // Stream pcd data 
    //ProcessPointClouds<pcl::PointXYZI> pointProcessorI; //Initialisation on stack
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>(); // Initialised on heap thus -> will be used 
    // std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1"); // For initialisation on stack
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2"); // For initialisation on heap

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI; //Place holder for cloud

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        // inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());  // For initialisation on stack
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string()); // For initialisation on heap
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}