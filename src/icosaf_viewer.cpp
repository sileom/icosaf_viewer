#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "imbs/imbs.hpp"

using namespace sensor_msgs;
using namespace message_filters;

using namespace std;

cv::Mat getColorDepth(const cv::Mat& depthImage) {

  cv::Mat colorMat(depthImage.rows, depthImage.cols, CV_8UC3);
  colorMat.setTo(cv::Scalar(0, 0, 0));

  for(int y = 0; y < depthImage.rows; y++) {
    for(int x = 0; x < depthImage.cols; x++) {      
      float d = depthImage.at<float>(y, x);
      if(d > 100.f && d < 2000.0f) {
	if(d < 200.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 191);
        }
	else if(d < 250.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 0, 255);
        }
        else if(d < 300.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 63, 255);
        }
        else if(d < 350.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 127, 255);
        }
        else if(d < 400.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 191, 255);
        }
        else if(d < 450.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(0, 255, 255);
        }
        else if(d < 500.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(63, 255, 191);
        }
        else if(d < 550.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(127, 255, 127);
        }
        else if(d < 600.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(191, 255, 63);
        }
        else if(d < 650.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 255, 0);
        }
        else if(d < 700.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 191, 0);
        }
        else if(d < 750.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 127, 0);
        }
        else if(d < 800.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 63, 0);
        }
        else if(d < 850.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(255, 0, 0);
        }
        else if(d < 900.f) {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(191, 0, 0);
        }
        else {
          colorMat.at<cv::Vec3b>(y,x) = cv::Vec3b(143, 0, 0);
        }
      }
    }
  }
  return colorMat;

}

void callback(const ImageConstPtr& depthImage_, const ImageConstPtr& rgbImage_)
{
    ROS_INFO("%s\n", "callback");

    cv_bridge::CvImagePtr rgbImage;
    try {
        rgbImage = cv_bridge::toCvCopy(rgbImage_, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_INFO("%s\n", "rgb");
    cv::imshow("RGB", rgbImage->image);

    cv_bridge::CvImageConstPtr depthImage;
    try {
        depthImage = cv_bridge::toCvShare(depthImage_, sensor_msgs::image_encodings::TYPE_32FC1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    

    ROS_INFO("%s\n", "depth");
    cv::Mat colorMat = getColorDepth(depthImage->image);

    
    ROS_INFO("%s\n", "color depth");
    cv::imshow("Color Depth", colorMat);
    //cv::imshow("Depth", depthImage->image);


    //IMBS Background Subtractor
    BackgroundSubtractorIMBS* pIMBS;
    pIMBS = new BackgroundSubtractorIMBS(30);
    pIMBS->loadBg("/home/labarea-franka/catkin_ws/src/icosaf_viewer/bgmodel/model.txt");
    
    cv::Mat frame = rgbImage->image;
    cv::Mat fgMask;
    //update the background model
    pIMBS->apply(frame, fgMask);
    //get background image
    cv::Mat bgImage;
    pIMBS->getBackgroundImage(bgImage);

    for(int i = 0; i < frame.rows; i++) {
        for(int j = 0; j < frame.cols; j++) {
            if(fgMask.at<uchar>(i,j) == 0) {
                frame.at<Vec3b>(i,j)[0] = 0;
                frame.at<Vec3b>(i,j)[1] = 0;
                frame.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }

    imshow("foreground", frame);
    imshow("BG Model", bgImage);

    cv::waitKey(30);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icosaf_viewer");

    //string topic_rgb = "camera/rgb/image_rect_color";
    string topic_rgb = "/camera/color/image_raw";
    //string topic_depth = "camera/depth_registered/sw_registered/image_rect";
    string topic_depth = "/camera/aligned_depth_to_color/image_raw";
	
    cout << "Subscriptions:" << endl;
    cout << "  - RGB topic: " << topic_rgb << endl;
    cout << "  - Depth topic: " << topic_depth << endl;

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> depth_sub(nh, topic_depth, 1);
    message_filters::Subscriber<Image> rgb_sub(nh, topic_rgb, 1);

    typedef sync_policies::ApproximateTime<Image, Image> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();

    return 0;
}

