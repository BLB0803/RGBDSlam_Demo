#include "base.h"
using namespace std;

SLAM::SLAM(ros::NodeHandle &n){
    _depthSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(n, "/camera/depth/image", 10);
    _rgbSubscriber = new message_filters::Subscriber<sensor_msgs::Image>(n, "/camera/rgb/image_color", 10);
    _sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100), *_rgbSubscriber, *_depthSubscriber);
    _sync->registerCallback(boost::bind(&SLAM::callBack, this, _1, _2));
}

/*********************************
 * Ros's call back func, needs two [sensor_msgs::Image] scriber with RGB image & Depth image.
 *********************************/
void SLAM::callBack(const sensor_msgs::ImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    CAMERA camera;
    camera.cx = 325.5; camera.cy = 253.5; camera.fx = 518.0; camera.fy = 519.0; camera.scale = 1000;
    FRAME frame;
    frame.rgb = msgToMat(rgb);
    frame.depth = msgToMat(depth);
    vector<cv::DMatch> goodMatches;

    detectKeyPointsAndDesp(frame);
    matchTwoFrame(_preFrame, frame, goodMatches);

    if(!_preFrame.kp.empty()){
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
        cout << "preFrame3dPoint: " << preFrame3dPoint.size() << endl;
        cout << "frame2dPoint: " << frame2dPoint.size() << endl;
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
 * For processing images, we should convert Ros's image message to OpenCV image.
 * Use this func to convert a [sensor_msgs::Image] to [cv::Mat].
 * Be careful with depth image's encoding while using.
 *********************************/
cv::Mat SLAM::msgToMat(const sensor_msgs::ImageConstPtr &msg){
    cv::Mat img;
    // For rgb image
    if(msg->encoding == "rgb8"){
        cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cvPtr->image;
    }
    // For depth image
    else if(msg->encoding == "32FC1"){
       cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
       img = floatToMono8(cvPtr->image);
    }
    else{cout << "Cant deal with " << msg->encoding << "format. " << endl; }
    return img;
}


/*********************************
 * ORB detector's input image should be a grayscale image.
 * Use this func to convert a [cv::Mat] from BGR8 to grayscale.
 *********************************/
cv::Mat SLAM::rgbTogray(cv::Mat rgb){
    cv::Mat gray;
    cv::cvtColor(rgb,gray,CV_BGR2GRAY);
    return gray;
}

/*********************************
 * Input image should be Mono8 image, while solving PnP.
 * Use this func to convert a [cv::Mat] from 32FC1 to Mono8(CV_8UC1).
 *********************************/
cv::Mat SLAM::floatToMono8(const cv::Mat& floatImg){
    cv::Mat mono8Img;
    if(mono8Img.rows != floatImg.rows || mono8Img.cols != floatImg.cols){
        mono8Img = cv::Mat(floatImg.size(), CV_8UC1);
    }
    cv::convertScaleAbs(floatImg, mono8Img, 100, 0.0);
    return mono8Img;
}


/*********************************
 * Wanna match two images, we should get keypoints &  descrip of every image.
 * Use this func to get [vector<cv::KeyPoint> kp] & [cv::Mat desp] for ur FRAME.
 *********************************/
void SLAM::detectKeyPointsAndDesp(FRAME& f){
    //Create ORB detector & descriptor
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    detector->setMaxFeatures(1000);
    // Tried to catch some error info here, but not work.
    // Considering callback wouldnt run when theres no msg recived.
    // They had thought all of things for u. :(
    try{
        detector->detect(rgbTogray(f.rgb), f.kp);
        descriptor->compute(rgbTogray(f.rgb), f.kp, f.desp);
        // Should convert descriptor to CV_32F format when using FlannBasedMatcher.
        // See https://stackoverflow.com/questions/29694490/flann-error-in-opencv-3
        f.desp.convertTo(f.desp, CV_32F);
    }
    catch (cv::Exception& e){
        cout << e.what() << endl;
    }
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
        cout<<"good matches="<<goodMatches.size()<<endl;
    }
}

cv::Point3f SLAM::point2dTo3d(cv::Point3f& p, CAMERA& c){
    cv::Point3f point;
    point.z = double(p.z) / c.scale;
    point.x = (p.x - c.cx) * point.z / c.fx;
    point.y = (p.y - c.cy) * point.z / c.fy;
    return point;
}




