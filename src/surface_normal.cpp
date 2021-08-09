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
    nodeHandle.getParam("surface_normal/epsAngle", epsAngle);
    nodeHandle.getParam("surface_normal/groundPlaneNormal", groundPlaneNormal);
    nodeHandle.getParam("surface_normal/horizontalPlaneAngleThresh", horizontalPlaneAngleThresh);
    horizontalPlaneNormal << groundPlaneNormal[0], groundPlaneNormal[1], groundPlaneNormal[2];

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
    ROS_INFO("init ConcaveHull init successfull");
}
    
void SurfaceNormal::pointingFinger_callback(const geometry_msgs::PointStampedPtr& pointStamped_msg){
    // TODO: get pointing finger ray in map coordinates
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
        ROS_DEBUG("Horizontal plane containing %d points from main point cloud", inliers->indices.size());
        if (inliers->indices.size() > minPlaneSize){    
            projectInliersOn2DPlane(pointCloud, inliers, coefficients, projectedPlane);
            publishPointCloud(projectedPlane, pointCloud_msg, groundPlane_publisher);         // publishing ground plane
            removeInliers(pointCloud, inliers);
            ROS_DEBUG("Removed ground plane containing %d points from main point cloud", inliers->indices.size());
        }
    } while (inliers->indices.size() > minPlaneSize && pointCloud->points.size() > minPlaneSize);
    
}

Eigen::Matrix<float, 2,4> SurfaceNormal::getVecSpanOfLine(Eigen::Vector3f & pointA, Eigen::Vector3f & pointB){
    /*  return matrix representing a vector span of a line in homogenous coordinates.
        inputs: pointA, pointB: two points in R3.
        output: matrix representing a line in R4.
    */
    
    Eigen::Vector4f p1, p2;
    Eigen::Matrix<float, 2,4> Wline(2, 4);
    p1 << pointA[0], pointA[1], pointA[2], 1;
    p2 << pointB[0], pointB[1], pointB[2], 1;

    Wline.row(0) = p1;
    Wline.row(1) = p2;

    return Wline;  
}

Eigen::Matrix<float, 2,4> SurfaceNormal::getOrthogonalPlanesContainingLine(Eigen::Matrix<float, 2, 4> Wline){
    /*  Return matrix of planes in homogenous coordinates.
        The planes form the rows of the output matrix.
        Calculation is the two eigenvectors with smallest signular values resulting from SVD of line span matrix
        (see: https://engineering.purdue.edu/kak/computervision/ECE661Folder/Lecture6.pdf)
    */
   using namespace Eigen;
 
   Matrix<float, 2,4>  Wplanes;
   MatrixXf U, S, V;

    // SVD decomposition
    JacobiSVD<Eigen::MatrixXf> svd(Wline, ComputeFullU | ComputeFullV);
    U = svd.matrixU();
    S = svd.singularValues();
    V = svd.matrixV();

    // get the two lowes vectors in V:
    Wplanes.row(0) = V.col(2);
    Wplanes.row(1) = V.col(3);

    return Wplanes;
}

Eigen::Vector4f SurfaceNormal::getIntersectionPointOf3Planes(Eigen::Vector4f plane1, Eigen::Vector4f plane2, Eigen::Vector4f plane3){
    /* the intersection point of three non parallel planes is a point.
    (see: https://engineering.purdue.edu/kak/computervision/ECE661Folder/Lecture6.pdf)
    */

    using namespace Eigen;
    Matrix<float, 3, 4> M;
    Vector4f x;
    Vector4f intersectionPoint;
    M << plane1, plane2, plane3;
    x << 0, 0, 0, 0;

    // check if there is a solution. rank(M) == M.rows;
    if (M.colPivHouseholderQr().rank() != M.rows()){
        intersectionPoint << NAN, NAN, NAN, NAN;
        return intersectionPoint;
    }

    intersectionPoint = M.colPivHouseholderQr().solve(x);
    
    return intersectionPoint;
   
}

Eigen::Vector4f SurfaceNormal::getInersectionLinePlane(Eigen::Vector3f & pointA, Eigen::Vector3f & pointB, pcl::ModelCoefficients::Ptr & planeCoeff){
    using namespace Eigen;
        Matrix<float, 2, 4> AB;
        Matrix<float, 2, 4> PQ;
        Vector4f plane1, plane2, plane3, intersectionPoint;

    AB = getVecSpanOfLine(pointA, pointB);
    PQ = getOrthogonalPlanesContainingLine(AB);
    plane1 = PQ.row(0);
    plane2 = PQ.row(1);
    plane3 << planeCoeff->values[0], planeCoeff->values[1], planeCoeff->values[2], planeCoeff->values[3];
                   
    intersectionPoint = getIntersectionPointOf3Planes(plane1, plane2, plane3);
    ROS_DEBUG("Intersection of pointing finger ray and plane [%.3f, %.3f, %.3f, %.3f]", intersectionPoint[0], intersectionPoint[1], 
                                                                                        intersectionPoint[2], intersectionPoint[3]);
    
    return intersectionPoint; 

}

void SurfaceNormal::pointCloud_callback(const sensor_msgs::PointCloud2Ptr& pointCloud_msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);   
    pcl::fromROSMsg(*pointCloud_msg, *pointCloud);

    ROS_DEBUG("New point cloud with %d points ", pointCloud->points.size());
    
    if (! isPointCloudValid(*pointCloud, minPointCloudSize)) {
        ROS_DEBUG("Not enough points in point cloud: %d < %d", (int)pointCloud->points.size(), (int)minPointCloudSize);
        return;
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr projectedPlane (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subsetToRemove (new pcl::PointCloud<pcl::PointXYZ>);   
    
    Eigen::Vector3f vecSurfaceNormal;

    // remove all horizontal planes in plane cloud
    removeHorizontalPlanes(pointCloud, pointCloud_msg);
    
    double theta;
    int counter = 0;
    using std::acos;
    
    while (pointCloud->points.size() > minPlaneSize) {        
        seg.setInputCloud(pointCloud);
        seg.segment(*inliers, *coefficients); 
        
        ROS_DEBUG("plane %.3fx + %.3fy + %.3fz + %.3f ", coefficients->values[0], coefficients->values[1],
                                                         coefficients->values[2], coefficients->values[3]);
        // check if the retrieved point cloud is large enough:
        if (! isPointCloudValid(*inliers, minPlaneSize)) {
            ROS_DEBUG("Not enough points in segmented plane: %d", inliers->indices.size());
            removeInliers(pointCloud, inliers);
            continue;
        }

        vecSurfaceNormal = getSurfaceNormalVector(coefficients);
        // calculate angle between plane and normal to horizontal plane ([0,1,0])
        theta = acos(vecSurfaceNormal.dot(horizontalPlaneNormal) / (vecSurfaceNormal.norm() * horizontalPlaneNormal.norm()));
        ROS_DEBUG("angle to horizontal plane %.3f [deg]", theta);
        
        if (abs(theta) * 180.0f / M_PI < horizontalPlaneAngleThresh){
            ROS_DEBUG("Discarding plane %.3f[deg] < %.3f[deg]", theta, horizontalPlaneAngleThresh);
            removeInliers(pointCloud, inliers);
            continue;
        }

        // Project segmented inliers onto calculated plane
        projectInliersOn2DPlane(pointCloud, inliers, coefficients, projectedPlane);
        publishPointCloud(projectedPlane, pointCloud_msg, cloud_publisher);

        // calculate point of intersection between pointing finger ray and plane
        intersectionPfrayPlane = getInersectionLinePlane(pFRay, coefficients)
        if (intersectionPoint.sum() == NAN){
        ROS_DEBUG("No single solution to intersection of pointing finger ray with plane");
        ROS_DEBUG("This is probably because pointing finger ray is in parralel with plane, or part of plane");
    }
        
        // Create a concave hull representation of the projected plane
        getConcavHull(projectedPlane, chull, cloudHull);
        publishPointCloud(cloudHull, pointCloud_msg, cloudHull_publisher);
        ROS_DEBUG("Cloud hull with %d points", cloudHull->points.size());


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

