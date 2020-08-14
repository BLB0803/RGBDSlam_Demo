/* A demo of slam with yolo to get cloud with its position.
 * See README.md for more details.
 * By blb, 2020.
*/
#include "frame.h"
using namespace std;

FRAME::FRAME(){}

FRAME::FRAME(const sensor_msgs::ImageConstPtr &rgbMsg, const sensor_msgs::ImageConstPtr &depthMsg){
    rgb = msgToMat(rgbMsg);
    depth = msgToMat(depthMsg);
    kp = detectKeyPoints(rgb);
    desp = computeDesp(rgb, kp);
    init = true;
}

FRAME& FRAME::operator =(const FRAME& f){
    if(this != &f){
        this->rgb = f.rgb;
        this->depth = f.depth;
        this->kp = f.kp;
        this->desp = f.desp;
        this->init = f.init;
    }
    return *this;
}

/*********************************
 * For processing images, we should convert Ros's image message to OpenCV image.
 * Use this func to convert a [sensor_msgs::Image] to [cv::Mat].
 * Be careful with depth image's encoding when use, ur encoding may NOT BE 32FC1.
 *********************************/
cv::Mat FRAME::msgToMat(const sensor_msgs::ImageConstPtr &msg){
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
 * Input image should be Mono8 image, while solving PnP.
 * Use this func to convert a [cv::Mat] from 32FC1 to Mono8(CV_8UC1).
 *********************************/
cv::Mat FRAME::floatToMono8(const cv::Mat& floatImg){
    cv::Mat mono8Img;
    if(mono8Img.rows != floatImg.rows || mono8Img.cols != floatImg.cols){
        mono8Img = cv::Mat(floatImg.size(), CV_8UC1);
    }
    cv::convertScaleAbs(floatImg, mono8Img, 100, 0.0);
    return mono8Img;
}

/*********************************
 * ORB detector's input image should be a grayscale image.
 * Use this func to convert a [cv::Mat] from BGR8 to grayscale.
 *********************************/
cv::Mat FRAME::rgbTogray(const cv::Mat& rgb){
    cv::Mat gray;
    cv::cvtColor(rgb,gray,CV_BGR2GRAY);
    return gray;
}

/*********************************
 * Wanna match two images, we should get keypoints &  descrip of every image.
 * Use these two func below to get [vector<cv::KeyPoint> kp] and [cv::Mat desp] for ur FRAME.
 * e.g. For a keypoint cv::KeyPoint kp :
 *     kp.angle : Direction of the point.
 *     kp.octave : In which interval (seems like in which octave, anyway..not important) we got the point.
 *     kp.pt : Coordinates in image of the point.
 *     kp.response : For describing how good the point is.
 *     kp.size : Just .. size of the point.
 *********************************/
vector<cv::KeyPoint> FRAME::detectKeyPoints(const cv::Mat& rgb){
    //Create ORB detector
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    detector->setMaxFeatures(1000);
    vector<cv::KeyPoint> keyPoints;
    // Tried to catch some error info here, but not work.
    // Considering callback wouldnt run when theres no msg recived.
    // They had thought all of things for u. :(
    try{
        detector->detect(rgbTogray(rgb), keyPoints);
    }
    catch (cv::Exception& e){
        cout << e.what() << endl;
    }
    return keyPoints;
}
// Func for computing descrip, needs source image and its keypoints vector at same time.
cv::Mat FRAME::computeDesp(const cv::Mat& rgb, vector<cv::KeyPoint>& kp){
    //Create ORB descriptor
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Mat desp;
    try{
        descriptor->compute(rgbTogray(rgb), kp, desp);
        // Should convert descriptor to CV_32F format when using FlannBasedMatcher for matching.
        // See https://stackoverflow.com/questions/29694490/flann-error-in-opencv-3
        desp.convertTo(desp, CV_32F);
    }
    catch (cv::Exception& e){
        cout << e.what() << endl;
    }
    return desp;
}
