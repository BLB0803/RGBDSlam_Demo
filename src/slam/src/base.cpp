/* A demo of slam with yolo to get cloud with its position.
 * See README.md for more details.
 * By blb, 2020.
*/
#include "base.h"
#include "frame.h"
using namespace std;

SLAM::SLAM(ros::NodeHandle &n){
    _depthSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(n, "/camera/depth/image", 10);
    _rgbSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(n, "/camera/rgb/image_color", 10);
    _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), *_rgbSubscriber, *_depthSubscriber);
    _sync->registerCallback(boost::bind(&SLAM::callBack, this, _1, _2));
}

SLAM::~SLAM(){
    delete _depthSubscriber;
    delete _rgbSubscriber;
    delete _sync;
}

/*********************************
 * Ros's callback func, needs two [sensor_msgs::Image] scriber with RGB image & Depth image.
 *********************************/
void SLAM::callBack(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    CAMERA camera;
    camera.cx = 325.5; camera.cy = 253.5; camera.fx = 518.0; camera.fy = 519.0; camera.scale = 1000;
    FRAME frame(rgb, depth);
    vector<cv::DMatch> goodMatches;   
    matchTwoFrame(_preFrame, frame, goodMatches);
    if(_preFrame.init){
        cv::Mat imgMatches;
        cv::drawMatches(_preFrame.rgb, _preFrame.kp, frame.rgb, frame.kp, goodMatches, imgMatches);
        cv::imshow( "good matches", imgMatches);
        cv::waitKey(1);



        vector<cv::Point3f> preFrame3dPoint;
        vector<cv::Point2f> frame2dPoint;
        for(int i = 0; i < goodMatches.size(); i++){
            cv::Point2f p = _preFrame.kp[goodMatches[i].queryIdx].pt;
            ushort d = _preFrame.depth.ptr<ushort>(int(p.y))[int(p.x)];
            if(d != 0){
                frame2dPoint.push_back(cv::Point2f(frame.kp[goodMatches[i].trainIdx].pt));
                cv::Point3f pt(p.x, p.y, d);
                cv::Point3f pd = point2dTo3d(pt, camera);
                preFrame3dPoint.push_back(pd);
            }
        }
        double cameraMatrixData[3][3] = {
            {camera.fx, 0, camera.cx},
            {0, camera.fy, camera.cy},
            {0, 0, 1}
        };
        if(preFrame3dPoint.size() > 6){
            cv::Mat cameraMatrix(3, 3, CV_64F, cameraMatrixData);
            cv::Mat rvec, tvec, inliers;
            cv::solvePnPRansac(preFrame3dPoint, frame2dPoint, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers);
            POSE pose;
            pose.rMat = rvec;
            pose.tMat = tvec;
            pose.inliers = inliers.rows;
            cout << "Pose: " << pose.inliers << endl;
        }
    }
    _preFrame = frame;
}


/*********************************
 * A func to match keypoints between two frames.
 * By matching keypoints, we could get camera pose's change.
 * Use this func to get [vector<cv::DMatch> goodMatches] of two frames:
 * Which has more than 4x minimum distance of all matches in [vector<cv::DMatch> matches].
 *********************************/
void SLAM::matchTwoFrame(FRAME& f1, FRAME& f2, vector<cv::DMatch>& goodMatches){
    if(!f1.kp.empty() && !f2.kp.empty()){
        vector<cv::DMatch> matches;
        cv::FlannBasedMatcher matcher;
        matcher.match(f1.desp, f2.desp, matches);
        double minDis = 9999;
        for(int i=0; i < matches.size(); i++){
            if (matches[i].distance < minDis) minDis = matches[i].distance;
        }
        for(int i=0; i < matches.size(); i++){
            if (matches[i].distance < 4 * minDis) goodMatches.push_back(matches[i]);
        }
    }
}

cv::Point3f SLAM::point2dTo3d(const cv::Point3f& p, const CAMERA& c){
    cv::Point3f point;
    point.z = double(p.z) / c.scale;
    point.x = (p.x - c.cx) * point.z / c.fx;
    point.y = (p.y - c.cy) * point.z / c.fy;
    return point;
}




