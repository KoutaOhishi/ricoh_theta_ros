#include "ros/ros.h"
#include "iostream"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/opencv_modules.hpp"
#include "image_transport/image_transport.h"


#define PI 3.1415926536

class PerspectiveConverter{
    private:
        ros::NodeHandle nh;

        std::string sub_img_name;
        std::string pub_img_name;

        ros::Subscriber sub_img;
        image_transport::Publisher pub_img;//image型のtopicのpublisherを宣言
        image_transport::Publisher front_img_pub;
        image_transport::Publisher back_img_pub;

    public:
        PerspectiveConverter(){
            this->sub_img_name = "/ricoh_theta/image_raw";
            this->pub_img_name = "/ricoh_theta/perspective/image_raw/";

            ROS_INFO("sub_img_name:[%s]", this->sub_img_name.c_str());

            this->sub_img = nh.subscribe(this->sub_img_name, 100, &PerspectiveConverter::sub_img_CB, this);

            image_transport::ImageTransport it(this->nh);
            this->pub_img = it.advertise(this->pub_img_name, 1);

            this->front_img_pub = it.advertise("/ricoh_theta/perspective/front_image_raw", 1);
            this->back_img_pub = it.advertise("/ricoh_theta/perspective/back_image_raw", 1);
        }

        cv::Point2f getInputPoint(int x, int y,int srcwidth, int srcheight)
        {
          cv::Point2f pfish;
          float theta,phi,r, r2;
          cv::Point3f psph;
          float FOV =(float)PI/180 * 180;
          float FOV2 = (float)PI/180 * 180;
          float width = srcwidth;
          float height = srcheight;

          // Polar angles
          theta = PI * (x / width - 0.5); // -pi/2 to pi/2
          phi = PI * (y / height - 0.5);  // -pi/2 to pi/2

          // Vector in 3D space
          psph.x = std::cos(phi) * std::sin(theta);
          psph.y = std::cos(phi) * std::cos(theta);
          psph.z = std::sin(phi) * std::cos(theta);

          // Calculate fisheye angle and radius
          theta = std::atan2(psph.z,psph.x);
          phi = std::atan2(std::sqrt(psph.x*psph.x+psph.z*psph.z),psph.y);

          r = width * phi / FOV;
          r2 = height * phi / FOV2;

          // Pixel in fisheye space
          pfish.x = 0.5 * width + r * std::cos(theta);
          pfish.y = 0.5 * height + r2 * std::sin(theta);
          return pfish;
        }

        cv::Mat fisheye2perspective(cv::Mat input)
        {
          //前後のカメラごとに画像をトリミング
          cv::Mat input_front_img(input, cv::Rect(10, 10, 610, 610));
          cv::Mat input_back_img(input, cv::Rect(660, 10, 610, 610));

          //展開用の画像を宣言
          cv::Mat front_img(input_front_img.rows, input_front_img.cols, CV_8UC3);
          cv::Mat back_img(input_back_img.rows, input_back_img.cols, CV_8UC3);

          //画像の回転
          cv::flip(input_front_img.t(), input_front_img, 1);
          cv::flip(input_back_img.t(), input_back_img, 0);

          //fisheyeの展開
          for( int i = 0; i < input_front_img.cols; i++ ){
            for( int j = 0; j < input_front_img.rows; j++ ){
              cv::Point2f inP2 = getInputPoint(i, j, input_front_img.cols, input_front_img.rows);
              cv::Point inP((int)inP2.x, (int)inP2.y);

              if(inP.x >= input_front_img.cols || inP.y >= input_front_img.rows)
                continue;

              if(inP.x < 0 || inP.y < 0)
                  continue;

              cv::Vec3b color = input_front_img.at<cv::Vec3b>(inP);
              front_img.at<cv::Vec3b>(cv::Point(i,j)) = color;
            }
          }

          for( int i = 0; i < input_back_img.cols; i++ ){
            for( int j = 0; j < input_back_img.rows; j++ ){
              cv::Point2f inP2 = getInputPoint(i, j, input_back_img.cols, input_back_img.rows);
              cv::Point inP((int)inP2.x, (int)inP2.y);

              if(inP.x >= input_back_img.cols || inP.y >= input_back_img.rows)
                continue;

              if(inP.x < 0 || inP.y < 0)
                  continue;

              cv::Vec3b color = input_back_img.at<cv::Vec3b>(inP);
              back_img.at<cv::Vec3b>(cv::Point(i,j)) = color;
            }
          }


          //出力用の画像を宣言
          cv::Mat result_img(input_front_img.rows, input_front_img.cols+input_back_img.cols, CV_8UC3);

          //back_cameraの画像を半分にする
          cv::Mat back_img_left(back_img, cv::Rect(0, 0, 305, 610));
          cv::Mat back_img_right(back_img, cv::Rect(305, 0, 305, 610));

          cv::Mat roi = result_img(cv::Rect(306, 0, front_img.cols, front_img.rows));
          front_img.copyTo(roi);

          roi = result_img(cv::Rect(915, 0, back_img_left.cols, back_img_left.rows));
          back_img_left.copyTo(roi);

          roi = result_img(cv::Rect(0, 0, back_img_right.cols, back_img_right.rows));
          back_img_right.copyTo(roi);


          sensor_msgs::ImagePtr front_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", front_img).toImageMsg();
          this->front_img_pub.publish(front_img_msg);
          sensor_msgs::ImagePtr back_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", back_img).toImageMsg();
          this->back_img_pub.publish(back_img_msg);

          return result_img;
        }

        void sub_img_CB(const sensor_msgs::ImageConstPtr& msg)
        {
            //ROS_INFO("Subscribed Image");

            //ROSのImage型からOpenCVで扱える形式に変換
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::Mat result_image;
            result_image = fisheye2perspective(image);


            //OpenCVの形式からROSのImage型に変換
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_image).toImageMsg();

            //publish
            this->pub_img.publish(img_msg);
        }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "perspective_converter");
    ROS_INFO("### START [PerspectiveConverter] ###");

    PerspectiveConverter pc;

    ros::spin();
}
