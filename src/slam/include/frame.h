/* A demo of slam with yolo to get cloud with its position.
 * See README.md for more details.
 * By blb, 2020. 
*/

#ifndef FRAME_H
#define FRAME_H
// std lib
#include <iostream>
#include <string>
#include <vector>

// ROS lib
#include <ros/ros.h>
#include "sensor_msgs/Image.h"

// OpenCV lib
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


using namespace std;

class FRAME{
public:
    FRAME();
    FRAME(const sensor_msgs::ImageConstPtr &rgbMsg, const sensor_msgs::ImageConstPtr &depthMsg);
    FRAME& operator =(const FRAME& f);
    cv::Mat rgb;
    cv::Mat depth;
    vector<cv::KeyPoint> kp;
    cv::Mat desp; 
    bool init = false;
private:
    cv::Mat msgToMat(const sensor_msgs::ImageConstPtr &msg);
    cv::Mat floatToMono8(const cv::Mat& floatImg);
    cv::Mat rgbTogray(const cv::Mat& rgb);
    vector<cv::KeyPoint> detectKeyPoints(const cv::Mat& rgb);
    cv::Mat computeDesp(const cv::Mat& rgb, vector<cv::KeyPoint>& kp);
         
};
#endif

