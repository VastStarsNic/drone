#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include "plugins/PluginBase.hpp"

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include "plugins/pid.hpp"
#include "whud_vision/ImageProcessingAction.h"

#include <qrencode.h>
#include <fstream>

#include <time.h>

using namespace cv;
using namespace std;

namespace vision_plugins {
typedef actionlib::SimpleActionServer<whud_vision::ImageProcessingAction>
    Server;
class QRCodeRecognize : public vision_base::VisionBase {
public:
  QRCodeRecognize();
  void initialize();

  // override interface function of basis class
  void UpdateImage(Mat image);

private:
  Mat image_now_;
  ros::NodeHandle n_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher img_pub_;

  zbar::ImageScanner scanner_;
  Server server_;

  bool server_cancel_;
  int recognize_freq_;

  bool if_get_code;

  void Code_locate();
  void execute(const whud_vision::ImageProcessingGoalConstPtr& goal);
};

QRCodeRecognize::QRCodeRecognize()
    : nh_("~QRCodeRecognize"),
      it_(nh_),
      server_(
          n_, "qrcode_recognize",
          boost::bind(&QRCodeRecognize::execute, this, boost::placeholders::_1),
          true) {
  image_now_ = Mat(2000, 2000, CV_8UC3, 1);
  img_pub_ = it_.advertise("img_result", 1);
}

void QRCodeRecognize::initialize() {
  n_.param<int>("/whud_vision_nodelet/QRCodeRecognize/recognize_freq",
                recognize_freq_, 10);

  if_get_code = false;
  scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
  ROS_INFO("QRCodeRecognize Init");
}

void QRCodeRecognize::UpdateImage(Mat image) {
  image_now_ = image.clone();
}

void QRCodeRecognize::Code_locate() {
  Mat codeImg = image_now_.clone();
  codeImg = codeImg(Range(720/2-540/2, 720/2+540/2), Range(1280/2-960/2, 1280/2+960/2));
  Mat grayImg;
  Mat result;
  Mat dstImg;
  GaussianBlur(codeImg, dstImg, Size(3, 3), 1);
  addWeighted(codeImg, 2, dstImg, -1, 0, codeImg);

  cvtColor(codeImg, grayImg, CV_BGR2GRAY);
  zbar::Image zbarImg(grayImg.cols, grayImg.rows, "Y800", (uchar*)grayImg.data,
                      grayImg.cols * grayImg.rows);
  int n = scanner_.scan(zbarImg);
  for (zbar::Image::SymbolIterator symbol = zbarImg.symbol_begin();
       symbol != zbarImg.symbol_end(); ++symbol) {
    vector<Point> rect_4points;
    if (symbol->get_location_size() == 4) {
      for (int i = 0; i < 4; i++) {
        rect_4points.push_back(
            Point(symbol->get_location_x(i), symbol->get_location_y(i)));
      }
      Point2f Point_CodeImg[3] = {rect_4points[0], rect_4points[1],
                                  rect_4points[2]};
      Point2f Point_DstImg[3] = {Point(0, 0), Point(0, 450), Point(450, 450)};
      Mat Trans = getAffineTransform(Point_CodeImg, Point_DstImg);
      warpAffine(codeImg, result, Trans, Size(450, 450));
      cvtColor(result, result, CV_BGR2GRAY);
      threshold(result, result, 140, 255, CV_THRESH_BINARY);

      medianBlur(result, result, 1);
      //medianBlur(result, result, 1);

      imwrite("/home/nvidia/Code/simple_ws/src/whud_vision/picture/QRCode.jpg", result);

      if_get_code = true;
      line(codeImg, rect_4points[0], rect_4points[1], Scalar(0, 255, 0), 2);
      line(codeImg, rect_4points[1], rect_4points[2], Scalar(0, 255, 0), 2);
      line(codeImg, rect_4points[2], rect_4points[3], Scalar(0, 255, 0), 2);
      line(codeImg, rect_4points[3], rect_4points[0], Scalar(0, 255, 0), 2);
    } else {
      imwrite("/home/nvidia/Code/simple_ws/src/whud_vision/picture/BarCode.jpg",
              codeImg);
      if_get_code = true;
    }
  }

  sensor_msgs::ImageConstPtr img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", codeImg).toImageMsg();
  img_pub_.publish(img_msg);
}

void QRCodeRecognize::execute(
    const whud_vision::ImageProcessingGoalConstPtr& goal) {
  if_get_code = false;
  ros::Rate rate(recognize_freq_);
  whud_vision::ImageProcessingFeedback feedback;
  while (!if_get_code) {
    if(server_.isPreemptRequested())
    {
      break;
    }
    Code_locate();
    rate.sleep();
  }
  ROS_INFO("CodeRecognize Execute Finish");
  server_.setSucceeded();
}

}  // namespace vision_plugins

PLUGINLIB_EXPORT_CLASS(vision_plugins::QRCodeRecognize, vision_base::VisionBase)
