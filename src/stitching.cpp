#include "ros/ros.h"
#include "iostream"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/opencv_modules.hpp"
#include "image_transport/image_transport.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "chrono"
#include "cmath"

#define PI 3.1415926536

class Img_subscriber{
    private:
        ros::NodeHandle nh;

        std::string sub_img_name;
        std::string pub_img_name;

        ros::Subscriber sub_img;
        image_transport::Publisher pub_img;//image型のtopicのpublisherを宣言
        image_transport::Publisher front_img_pub;
        image_transport::Publisher back_img_pub;

    public:
        Img_subscriber(){
            this->sub_img_name = "/ricoh_theta/image_raw";
            this->pub_img_name = "/ricoh_theta/perspective/image_raw/";

            ROS_INFO("sub_img_name:[%s]", this->sub_img_name.c_str());

            this->sub_img = nh.subscribe(this->sub_img_name, 100, &Img_subscriber::sub_img_CB, this);

            image_transport::ImageTransport it(this->nh);
            this->pub_img = it.advertise(this->pub_img_name, 1);

        }

        cv::Mat fisheye2perspective(cv::Mat input) //input 1280x720
        {
          std::chrono::system_clock::time_point  start, end; // 型は auto で可
          start = std::chrono::system_clock::now(); // 計測開始時間

          double vertex = 640;
          double src_cx = 320;
          double src_cy = 320;
          double src_r = 283;
          double src_cx2 = 1280 - src_cx;

          cv::Mat map_x = cv::Mat::zeros(vertex, vertex*2, CV_32FC2);
          cv::Mat map_y = cv::Mat::zeros(vertex, vertex*2, CV_32FC2);

          //double map_x_array[vertex*2][vertex];
          //double map_y_array[vertex*2][vertex];

          for( int y = 0; y < vertex; y++ ){
            unsigned char* ptr_x = map_x.ptr( y );
            unsigned char* ptr_y = map_y.ptr( y );
            for( int x = 0; x < vertex*2 ; x++ ){
              double phi1 = PI * x / vertex;
              double theta1 = PI * y / vertex;

              double X = std::sin(theta1) * std::cos(phi1);
              double Y = std::sin(theta1) * std::sin(phi1);
              double Z = std::cos(theta1);

              double phi2 = std::acos(-X);

              double theta2;
              if( Y > 0 ){ theta2 = std::acos(-Z/std::sqrt(Y*Y + Z*Z)); }
              else if( Y == 0){ theta2 = 0; }
              else{ theta2 = -1.0 * std::acos(-Z/std::sqrt(Y*Y + Z*Z)); }


              // 0 等距離射影
              // 1 立体射影
              // 2 立体射影逆変換
              // 3 正射影
              // 4 正射影逆変換
              int method = 0;

              double r_;
              if( phi2 < PI/2 ){
                if( method == 0 ){
                  r_ = phi2 / PI*2;
                }

                else if( method == 1 ){
                  r_ = std::tan(phi2/2);
                }

                else if( method == 2 ){
                  r_ = 1 - std::tan((PI/2-phi2)/2);
                }

                else if( method == 3 ){
                  r_ = std::sin(phi2);
                }

                else if( method == 4 ){
                  r_ = 1 - std::sin(PI/2-phi2);
                }
                double x_ = src_r * r_ * std::cos(theta2) + src_cx;
                double y_ = src_r * r_ * std::sin(theta2) + src_cy;
                ptr_x[x] = x_;
                ptr_y[x] = y_;
                //map_x.at<unsigned char>(y, x) = src_r * r_ * std::cos(theta2) + src_cx;
                //map_y.at<unsigned char>(y, x) = src_r * r_ * std::sin(theta2) + src_cy;
                //map_x_array[y][x] = x_;
                //map_y_array[y][x] = y_;
              }

              else{
                if( method == 0 ){
                  r_ = (PI - phi2) / PI * 2;
                }

                else if( method == 1 ){
                  r_ =  std::tan((PI - phi2) / 2);
                }

                else if( method == 2 ){
                  r_ =  1 - std::tan((-PI/2 + phi2) / 2);
                }

                else if( method == 3 ){
                  r_ = std::sin(PI - phi2);
                }

                else if( method == 4 ){
                  r_ = 1 - std::sin(-PI / 2 + phi2);
                }
                double x_ = src_r * r_ * std::cos(PI - theta2) + src_cx2;
                double y_ = src_r * r_ * std::sin(PI - theta2) + src_cy;
                ptr_x[x] = x_;
                ptr_y[x] = y_;
                //map_x.at<unsigned char>(y, x) = src_r * r_ * std::cos(PI - theta2) + src_cx2;
                //map_y.at<unsigned char>(y, x) = src_r * r_ * std::sin(PI - theta2) + src_cy;
                //map_x_array[y][x] = x_;
                //map_y_array[y][x] = y_;
              }
            }//x
          }//y

          //map_x.convertTo(map_x, CV_32FC1);
          //map_y.convertTo(map_y, CV_32FC1);

          cv::Mat output(vertex*2, vertex, CV_8UC3);
          //cv::Mat output = input.clone();
          //cv::remap(input, output, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

          for( int y = 0; y < vertex; y++ ){
            unsigned char* ptr_map_x = map_x.ptr( y );
            unsigned char* ptr_map_y = map_y.ptr( y );
            unsigned char* ptr_input = input.ptr( y );
            unsigned char* ptr_output = output.ptr( y );
            for( int x = 0; x < vertex*2 ; x++ ){
              auto x_ = ptr_map_x[x];
              auto y_ = ptr_map_y[x];
              ptr_output[x] = input.at<unsigned char>(x_, y_);
            }
          }
          cv::flip(output.t(), output, 0);



          //std::cout << map_x_array[0][0] << std::endl;
          //std::cout << map_y_array[0][0] << std::endl;

          end = std::chrono::system_clock::now();  // 計測終了時間
          double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count(); //処理に要した時間
          float fps = 1000.00 / elapsed;
          printf("fps:[%f]\n", elapsed);

          return output;
        }

        void sub_img_CB(const sensor_msgs::ImageConstPtr& msg)
        {
            //ROS_INFO("Subscribed Image");

            //ROSのImage型からOpenCVで扱える形式に変換
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            cv::Mat result_img;
            result_img = fisheye2perspective(image);
            cv::imshow("result", result_img);
            cv::waitKey(1);

            //OpenCVの形式からROSのImage型に変換
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result_img).toImageMsg();

            //publish
            this->pub_img.publish(img_msg);
        }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "stitching");
    ROS_INFO("### START [img_subscriber] ###");

    Img_subscriber i;

    ros::spin();
}
