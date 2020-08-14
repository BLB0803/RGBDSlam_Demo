/* A demo of slam with yolo to get cloud with its position.
 * See README.md for more details.
 * By blb, 2020. 
*/

#ifndef BASE_H
#define BASE_H
// std lib
#include <iostream>
#include <string>
#include <vector>

// ROS lib
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV lib
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL lib
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

// Eigen3 lib
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// My lib
#include "frame.h"
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

struct CAMERA{
    double cx, cy, fx, fy;
    int scale; 
};

struct POSE{
    cv::Mat rMat, tMat;
    int inliers; 
};

class SLAM{
public:
    SLAM(ros::NodeHandle &n);
    ~SLAM();
    
private:
    void callBack(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth);
    void matchTwoFrame(FRAME& f1, FRAME& f2, vector<cv::DMatch>& goodMatches);
    cv::Point3f point2dTo3d(const cv::Point3f& p, const CAMERA& c);
    
    message_filters::Subscriber<sensor_msgs::Image>* _depthSubscriber;    
    message_filters::Subscriber<sensor_msgs::Image>* _rgbSubscriber;
    message_filters::Synchronizer<MySyncPolicy>* _sync;
    FRAME _preFrame;

};
#endif

