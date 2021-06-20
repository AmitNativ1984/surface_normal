#include "surface_normal.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
// using namespace std;

SurfaceNormal::SurfaceNormal(ros::NodeHandle & nodeHandle){
    
    //get parameters from launch file
    nodeHandle.getParam("surface_normal/minPointCloudSize", minPointCloudSize);
    nodeHandle.getParam("surface_normal/depthError", depthError);
    nodeHandle.getParam("surface_normal/maxIterations", maxIterations);
    nodeHandle.getParam("surface_normal/minPlaneSize", minPlaneSize);
    nodeHandle.getParam("surface_normal/alpha", alpha);
    nodeHandle.getParam("surface_normal/groundPlaneNormal", groundPlaneNormal);
    nodeHandle.getParam("surface_normal/epsAngle", epsAngle);

    initSubscribers(nodeHandle);
    initPublishers(nodeHandle);
    initGroundSegmentor(groundSeg);
    initSegmentor(seg);
    initConcaveHull(chull);

}

void SurfaceNormal::initSubscribers(ros::NodeHandle & nh){
    pointingFinger_subscriber = nh.subscribe("/pointingfinger/TargetPoint", 1, &SurfaceNormal::pointingFinger_callback, this);
    ROS_INFO("pointingFinger subscriber init successfull");
    pointCloud_subscriber = nh.subscribe("/d415/depth/color/points", 1, &SurfaceNormal::pointCloud_callback, this);
    ROS_INFO("pointCloud subscriber init successfull");
}

void SurfaceNormal::initPublishers(ros::NodeHandle & nh){
    cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surface_normal/cloud",1);
    ROS_INFO("cloud_publisher init successfull");    

    selectedCloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surface_normal/cloud_selected",1);
    ROS_INFO("selectedCloud_publisher init successfull");    

    groundPlane_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surface_normal/cloud_ground_plane",1);
    ROS_INFO("groundPlane_publisher init successfull");    

    cloudHull_publisher = nh.advertise<sensor_msgs::PointCloud2>("/surface_normal/cloud_hull",1);
    ROS_INFO("cloudHull_publisher publisher init successfull");
}

void SurfaceNormal::initGroundSegmentor(pcl::SACSegmentation<pcl::PointXYZ> &seg ) {
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(depthError);
    seg.setMaxIterations(maxIterations);
    Eigen::Vector3f axis(groundPlaneNormal[0], groundPlaneNormal[1], groundPlaneNormal[2]);
    seg.setAxis(axis);
    seg.setEpsAngle(epsAngle * (M_PI / 180.0f)); //plane can be within predefined degress of X-Y plane
    ROS_INFO("init ground plane segmentor successfull");
}

void SurfaceNormal::initSegmentor(pcl::SACSegmentation<pcl::PointXYZ> &seg ) {
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(depthError);
    seg.setMaxIterations(maxIterations);
    // Eigen::Vector3f axis(groundPlaneNormal[0], groundPlaneNormal[1], groundPlaneNormal[2]);
    // seg.setAxis(axis);
    seg.setEpsAngle(epsAngle * (M_PI / 180.0f)); //plane can be within 30 degress of X-Y plane
    ROS_INFO("init segmentor successfull");
}

void SurfaceNormal::initConcaveHull(pcl::ConcaveHull<pcl::PointXYZ> &chull){
    chull.setAlpha(alpha);
    ROS_INFO("initConcaveHull init successfull");
}
    
void SurfaceNormal::pointingFinger_callback(const geometry_msgs::PointStampedPtr& pointStamped_msg){
    pointingFingerCamOptXYZ = *pointStamped_msg;
    ROS_INFO_STREAM(pointingFingerCamOptXYZ);
}

bool SurfaceNormal::isPointCloudValid(pcl::PointCloud<pcl::PointXYZ>& pointCloud, const int thres){
    if (pointCloud.points.size() < thres) {
        return false;
    }
    else {
        return true;
    }
}

bool SurfaceNormal::isPointCloudValid(pcl::PointIndices& pointCloudIndices, const int thres){
    if (pointCloudIndices.indices.size() < thres) {
        return false;
    }
    else {
        return true;
    }
}

void SurfaceNormal::projectInliersOn2DPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr & inputPointCloud, pcl::PointIndices::Ptr & pointIndices, pcl::ModelCoefficients::Ptr & modelCoeff, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudProjected) {
    
    pcl::ProjectInliers<pcl::PointXYZ> projectInliers;
    projectInliers.setInputCloud(inputPointCloud);
    projectInliers.setModelType(pcl::SACMODEL_PLANE);
    projectInliers.setIndices(pointIndices);
    projectInliers.setModelCoefficients(modelCoeff);
    // get points associated with the plane surface
    projectInliers.filter(*cloudProjected);
    ROS_DEBUG("[projected inliers on 2D plane] modelCoeff: %.3f, %.3f, %.3f, %.3f, projected points size: %d", modelCoeff->values[0], modelCoeff->values[1], 
                                                                                                              modelCoeff->values[2], modelCoeff->values[3],
                                                                                                              cloudProjected->points.size());  
                                                          
}

void SurfaceNormal::getConcavHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected, pcl::ConcaveHull<pcl::PointXYZ> &chull, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull) {
    chull.setInputCloud(cloudProjected);
    chull.reconstruct(*cloudHull);
}

void SurfaceNormal::removeInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr & pointCloud, pcl::PointIndices::Ptr & inliers) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subsetToRemove (new pcl::PointCloud<pcl::PointXYZ>);  
    
    // remove segmented subset of plane from point cloud
    extract.setInputCloud(pointCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*subsetToRemove);
    pointCloud.swap(subsetToRemove);      
    ROS_DEBUG("Removed %d points from plane cloud", inliers->indices.size());
}

void SurfaceNormal::publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud, const sensor_msgs::PointCloud2Ptr &pointCloud_msg, ros::Publisher &publisher){
    // publishing projected plane
    sensor_msgs::PointCloud2 projectedPlane_ROS;
    pcl::toROSMsg(*pointCloud, projectedPlane_ROS);
    projectedPlane_ROS.header.frame_id = pointCloud->header.frame_id;
    projectedPlane_ROS.header.stamp = pointCloud_msg->header.stamp;
    publisher.publish(projectedPlane_ROS);
}

void SurfaceNormal::removeHorizontalPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr & pointCloud, const sensor_msgs::PointCloud2Ptr &pointCloud_msg){
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPlane (new pcl::PointCloud<pcl::PointXYZ>);
    // segment and remove horizontal planes:
    do {
        groundSeg.setInputCloud(pointCloud);
        groundSeg.segment(*inliers, *coefficients);
        if (inliers->indices.size() > minPlaneSize){    
            projectInliersOn2DPlane(pointCloud, inliers, coefficients, projectedPlane);
            publishPointCloud(projectedPlane, pointCloud_msg, groundPlane_publisher);         // publishing ground plane
            removeInliers(pointCloud, inliers);
            ROS_DEBUG("Removed ground plane containing %d points from main point cloud", inliers->indices.size());
        }
    } while (inliers->indices.size() > minPlaneSize && pointCloud->points.size() > minPlaneSize);
    
}

void SurfaceNormal::pointCloud_callback(const sensor_msgs::PointCloud2Ptr& pointCloud_msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::fromROSMsg(*pointCloud_msg, *pointCloud);

    ROS_DEBUG("New point cloud with %d points ", pointCloud->points.size());
    
    if (! isPointCloudValid(*pointCloud, minPointCloudSize)) {
        ROS_FATAL("Not enough points in point cloud %d", (int)pointCloud->points.size());
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPlaneHull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subsetToRemove (new pcl::PointCloud<pcl::PointXYZ>);   
    
    // remove all horizontal planes in plane cloud
    removeHorizontalPlanes(pointCloud, pointCloud_msg);
    
    int counter = 0;
    while (pointCloud->points.size() > minPlaneSize) {        
        seg.setInputCloud(pointCloud);
        seg.segment(*inliers, *coefficients); 
        
        // check if the retrieved point cloud is large enough:
        if (! isPointCloudValid(*inliers, minPlaneSize)) {
            ROS_DEBUG("Not enough points in segmened plane: %d", inliers->indices.size());
            return;
        }

        // Project segmented inliers onto calculated plane
        projectInliersOn2DPlane(pointCloud, inliers, coefficients, projectedPlane);
        
        // Create a concave hull representation of the projected plane
        getConcavHull(projectedPlane, chull, projectedPlaneHull);
        
        publishPointCloud(projectedPlane, pointCloud_msg, cloud_publisher);

        if (counter == 0){
            publishPointCloud(projectedPlane, pointCloud_msg, selectedCloud_publisher);
        }

        counter++;

        removeInliers(pointCloud, inliers);

        ROS_DEBUG("Point cloud left with %d points ", pointCloud->points.size());
    }
    
    /*
    1. convert pointing finger to camera frame pixels
    3. for every segmented surface:
        3.1 calculate 2D convex hull in image plane
        3.2 where ever convex hull reaches image frame, extened it to infinity
        3.3 check if pointing finger is inside or outside convex hull
        3.4 if outside:
            3.4.1 continue
        3.5 else if inside:
            3.5.1 if MATCHING_SURFACE is not NULL:
                3.5.1.1 don't report any surface (solution ambiguis) and exit with warning
            3.5.2 else: 
                MATCHING_SURFACE = current surface

    4. if MATCHING_SURFACE != NULL:
        4.1 calculate matching segmented surface 3D normal vector
        4.2. create infinte ray from pointing finger
        4.3. distance to target is the size of the intersection vector between pointing finger and surface
        4.4. return surface normal, distance to normal
    

    */



}

int main(int argc, char **argv){
    //init ros node:
    ros::init(argc, argv, "surface_normal");
    ros::NodeHandle nodeHandle;
    SurfaceNormal surfaceNormal = SurfaceNormal(nodeHandle);
    //while (ros::ok()){
    ros::spin();
    //}
    

    ROS_INFO("EXIT");
}   