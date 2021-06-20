#ifndef SURFACE_NORMAL_H
#define SURFACE_NORMAL_H

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>

class SurfaceNormal{
    
    private:
        int minPointCloudSize;
        double depthError;
        int maxIterations; 
        int minPlaneSize;
        float alpha;
        std::vector<double> groundPlaneNormal;
        double epsAngle;

        ros::Subscriber pointingFinger_subscriber, 
                        pointCloud_subscriber;

        ros::Publisher  surfaceNormal_publisher,
                        cloud_publisher,
                        groundPlane_publisher,
                        selectedCloud_publisher,
                        cloudHull_publisher;

        geometry_msgs::PointStamped pointingFingerCamOptXYZ;

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::SACSegmentation<pcl::PointXYZ> groundSeg;
        pcl::PointCloud<pcl::PointXYZ> *cloudProjectedPtr = new(pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConcaveHull<pcl::PointXYZ> chull;

    public:
        SurfaceNormal(ros::NodeHandle & nodeHandle);
        
        void getConfigParameters(ros::NodeHandle & nodeHandle);

        void initSubscribers(ros::NodeHandle & nh);
        
        void initPublishers(ros::NodeHandle & nh);
        
        void initGroundSegmentor(pcl::SACSegmentation<pcl::PointXYZ> &seg );
        
        void initSegmentor(pcl::SACSegmentation<pcl::PointXYZ> &seg ); 
        
        void initConcaveHull(pcl::ConcaveHull<pcl::PointXYZ> &chull);
        
        void pointingFinger_callback(const geometry_msgs::PointStampedPtr& pointStamped_msg);
        
        bool isPointCloudValid(pcl::PointCloud<pcl::PointXYZ>& pointCloud, const int thres);
        
        bool isPointCloudValid(pcl::PointIndices& pointCloudIndices, const int thres);
        
        void projectInliersOn2DPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr & inputPointCloud, pcl::PointIndices::Ptr & pointIndices, pcl::ModelCoefficients::Ptr & modelCoeff, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudProjected); 
        
        void getConcavHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProjected, pcl::ConcaveHull<pcl::PointXYZ> &chull, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudHull); 
        
        void removeInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr & pointCloud, pcl::PointIndices::Ptr & inliers);
        
        void publishPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & pointCloud, const sensor_msgs::PointCloud2Ptr& pointCloud_msg, ros::Publisher &publisher );
        
        void removeHorizontalPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr & pointCloud, const sensor_msgs::PointCloud2Ptr &pointCloud_msg);
    
        void pointCloud_callback(const sensor_msgs::PointCloud2Ptr& pointCloud_msg);

};

        
#endif