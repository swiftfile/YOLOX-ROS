#include <math.h>
#include <chrono>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

#include <bboxes_ex_msgs/BoundingBox.h>
#include <bboxes_ex_msgs/BoundingBoxes.h>
#include "../../../yolox_cpp/include/yolox_cpp/core.hpp"

namespace yolox_ros_cpp {

    class YoloXNode {
    public:
        YoloXNode(const std::string &node_name);

    private:
        void initializeParameter();

        std::unique_ptr<yolox_cpp::AbcYoloX> yolox_;
        std::string model_path_ = "src/YOLOX-ROS/weights/openvino/yolox_tiny.xml";
        std::string model_type_ = "tensorrt";
        std::string model_version_ = "0.1.1rc0";
        std::string device_ = "cpu";
        float conf_th_ = 0.3f;
        float nms_th_ = 0.45f;
        int image_width_;
        int image_height_;
        ros::NodeHandle nh;

        std::string src_image_topic_name_;
        std::string publish_image_topic_name_ = "yolox/image_raw";
        std::string publish_boundingbox_topic_name_ = "yolox/bounding_boxes";

        image_transport::Subscriber sub_image_;

//        void colorImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& ptr);
        void colorImageCallback(const sensor_msgs::Image::ConstPtr &msg);

//        rclcpp::Publisher <bboxes_ex_msgs::msg::BoundingBoxes>::SharedPtr pub_bboxes_;
        ros::Subscriber image_sub = nh.subscribe("/image_raw",10,&YoloXNode::colorImageCallback,this);
        ros::Publisher pub_bboxes = nh.advertise<bboxes_ex_msgs::BoundingBoxes>("bboxes", 1000);
        ros::Publisher pub_image_=nh.advertise<sensor_msgs::Image>("/yolox/images",10);

        bboxes_ex_msgs::BoundingBoxes
        objects_to_bboxes(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::Header header);

        std::string WINDOW_NAME_ = "YOLOX";
        bool imshow_ = true;
    };
}
