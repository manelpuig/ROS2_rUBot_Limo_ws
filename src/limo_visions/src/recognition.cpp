#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/image.h>
#include <image_transport/image_transport.hpp>
#include <geometry_msgs/msg/pose.hpp>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;
using std::placeholders::_1;

sensor_msgs::msg::Image hsv_image;

class Recognition : public rclcpp::Node
{
    public: Recognition()
    : Node("Recognition")
        {
        sub_img = image_transport::create_subscription(this,"/camera/color/image_raw",
                std::bind(&Recognition::imageCB,this,_1),
                "raw",rmw_qos_profile_sensor_data);
        pub_img = image_transport::create_publisher(this,"/image_hsv",rmw_qos_profile_sensor_data);

        }
    private:

        void imageCB(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
        {
            cv_bridge::CvImageConstPtr cvImage;

            try
            {   //使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
                cvImage = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);

            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            cv::Mat img = cvImage->image;
            cv::Mat gray;
            cv::cvtColor(img,  gray, cv::COLOR_RGB2GRAY);
                
                // 颜色空间转换
            Mat hsv;
            cvtColor(img, hsv, COLOR_BGR2HSV);

            // 颜色阈值  //使用inRange过滤像素
            Mat mask;
            // 黄色
            // inRange(hsv, Scalar(5, 110, 90), Scalar(50, 207, 220), mask);
            inRange(hsv, Scalar(124, 45, 100), Scalar(140, 255, 255), mask);
        

            // 形态学操作  开闭操作
            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
            morphologyEx(mask, mask, MORPH_OPEN, kernel);
            morphologyEx(mask, mask, MORPH_CLOSE, kernel);

            // 轮廓检测
            vector<vector<Point>> contours;
            findContours(mask.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            // 形状匹配
            for (size_t i = 0; i < contours.size(); i++)
            {
                double match = matchShapes(contours[i], contours[0], CONTOURS_MATCH_I1, 0);
                double area = cv::contourArea(contours[i], false);  //计算轮廓的面积
                if (area < 300)
                    continue;
                if (match < 0.1)
                {
                    Rect rect = boundingRect(contours[i]);
                    cv::Point p(rect.x, rect.y-10);
                    rectangle(img, rect, Scalar(0, 255, 0), 2);
                    cv::putText(img, "purple", p, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2, 8);
                }
            }
                
                // cv::cvtColor(img,img,cv::COLOR_HSV2BGR);
                sensor_msgs::msg::Image::ConstPtr  hsv_image_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                hsv_image = *hsv_image_;
                pub_img.publish(hsv_image);
                // cv::imshow("img", img);
                
                // char k = cv::waitKey(30);
                // if(k == 'q')
                //     return ;
            }

        image_transport::Publisher pub_img;
        image_transport::Subscriber sub_img;
        
};


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    //rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("move_to_ar");
   

    rclcpp::spin(std::make_shared<Recognition>());
    rclcpp::shutdown();
    return 0;
}


