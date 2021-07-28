#include <whud_vision/ImageProcessingAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/mcc.hpp>
#include <iostream>
using namespace std;
using namespace cv;
using namespace mcc;
using namespace ccm;
typedef actionlib::SimpleActionClient<whud_vision::ImageProcessingAction> Client;

void doneCb(const actionlib::SimpleClientGoalState& state,const whud_vision::ImageProcessingResultConstPtr& result)
{
    ROS_INFO("Done");
}

void activeCb()
{
    ROS_INFO("Start");
}

void feedbackCb(const whud_vision::ImageProcessingFeedbackConstPtr& feedback)
{
    ROS_INFO("x: %f, y: %f",feedback->twist.linear.x,feedback->twist.linear.y);
    ROS_INFO("angle_z: %f",feedback->twist.angular.z);
}

// int main(int argc , char** argv)
// {
//     ros::init(argc, argv, "action_test_client");
//     Client client("rplidar_measure",true);
//     ROS_INFO("Wait for server");
//     client.waitForServer();
//     ROS_INFO("Send goal");
//     whud_vision::ImageProcessingGoal goal;
//     goal.start = true;
//     client.sendGoal(goal,&doneCb,&activeCb,&feedbackCb);
//     ros::spin();
//     return 0;
// }
int main(int argc, char *argv[])
{
   // [get_messages_of_image]：获取图像消息
   string filepath = "input.png"; // 输入图片路径
   Mat image = imread(filepath, IMREAD_COLOR);
   Mat imageCopy = image.clone();
   Ptr<CCheckerDetector> detector = CCheckerDetector::create();
   // [get_color_checker]：准备ColorChecker检测
   vector<Ptr<mcc::CChecker>> checkers = detector->getListColorChecker();
   for (Ptr<mcc::CChecker> checker : checkers)
   {
       // [create]：创建CCheckerDetector对象，并使用getListColorChecker函数获取ColorChecker信息。
       Ptr<CCheckerDraw> cdraw = CCheckerDraw::create(checker);
       cdraw->draw(image);
       Mat chartsRGB = checker->getChartsRGB();
       Mat src = chartsRGB.col(1).clone().reshape(3, chartsRGB.rows/3);
       src /= 255.0;
       // [get_ccm_Matrix]：对于每个ColorChecker，都可以计算一个ccm矩阵以进行颜色校正。Model1是ColorCorrectionModel类的对象，可以根据需要来修改参数以获得最佳色彩校正效果。
       ColorCorrectionModel model1(src, COLORCHECKER_Vinyl);
       model1.run();
       Mat ccm = model1.getCCM();
       std::cout<<"ccm "<<ccm<<std::endl;
       double loss = model1.getLoss();
       std::cout<<"loss "<<loss<<std::endl;
 
       // [make_color_correction]：成员函数infer_image用于使用ccm矩阵进行校正校正。
       Mat img_;
       cvtColor(image, img_, COLOR_BGR2RGB);
       img_.convertTo(img_, CV_64F);
       const int inp_size = 255;
       const int out_size = 255;
       img_ = img_ / inp_size;
       Mat calibratedImage= model1.infer(img_);
       Mat out_ = calibratedImage * out_size;
       // [Save_calibrated_image]：保存已校准的图像。
       out_.convertTo(out_, CV_8UC3);
       Mat img_out = min(max(out_, 0), out_size);
       Mat out_img;
       cvtColor(img_out, out_img, COLOR_RGB2BGR);
       imwrite("output.png",out_img);
   }
 
   return 0;
}