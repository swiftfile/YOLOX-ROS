#include "../include/yolox_ros_cpp/yolox_ros_cpp.hpp"
#include "../../yolox_cpp/include/yolox_cpp/utils.hpp"

namespace yolox_ros_cpp {

//    YoloXNode::YoloXNode(const rclcpp::NodeOptions& options)
//    : YoloXNode::YoloXNode("", options)
//    {}

    YoloXNode::YoloXNode(const std::string &node_name) {
        ROS_INFO("initialize");
//        this->initializeParameter();

        if (this->imshow_) {
            char window_name[50];
//            sprintf(window_name, "%s %s %s", this->WINDOW_NAME_.c_str(), "_", this->get_name());
            this->WINDOW_NAME_ = window_name;

            cv::namedWindow(this->WINDOW_NAME_, cv::WINDOW_AUTOSIZE);
        }

        if (this->model_type_ == "tensorrt") {
#ifdef ENABLE_TENSORRT
            RCLCPP_INFO(this->get_logger(), "Model Type is TensorRT");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXTensorRT>(this->model_path_, std::stoi(this->device_),
                                                                      this->nms_th_, this->conf_th_, this->model_version_);
#else
            ROS_WARN("yolox_cpp is not built with TensorRT");
            ros::shutdown();
#endif
        } else if (this->model_type_ == "openvino") {
#ifdef ENABLE_OPENVINO
            RCLCPP_INFO(this->get_logger(), "Model Type is OpenVINO");
            this->yolox_ = std::make_unique<yolox_cpp::YoloXOpenVINO>(this->model_path_, this->device_,
                                                                      this->nms_th_, this->conf_th_, this->model_version_);
#else
            ROS_ERROR("yolox_cpp is not built with OpenVINO");
            ros::shutdown();
#endif
        }



//        this->sub_image_ = image_transport::create_subscription(
//                this, this->src_image_topic_name_,
//                std::bind(&YoloXNode::colorImageCallback, this, std::placeholders::_1),
//                "raw");
//        this->pub_bboxes_ = this->create_publisher<bboxes_ex_msgs::BoundingBoxes>(
//                this->publish_boundingbox_topic_name_,
//                10
//        );
//        this->pub_image_ = image_transport::create_publisher(this, this->publish_image_topic_name_);

    }

    void YoloXNode::initializeParameter()
    {
//        imshow_
//        this->declare_parameter<bool>("imshow_isshow", true);
//        this->declare_parameter<std::string>("model_path", "src/YOLOX-ROS/weights/openvino/yolox_tiny.xml");
//        this->declare_parameter<float>("conf", 0.3f);
//        this->declare_parameter<float>("nms", 0.45f);
//        this->declare_parameter<std::string>("device", "CPU");
//        this->declare_parameter<std::string>("model_type", "openvino");
//        this->declare_parameter<std::string>("model_version", "0.1.1rc0");
//        this->declare_parameter<std::string>("src_image_topic_name", "image_raw");
//        this->declare_parameter<std::string>("publish_image_topic_name", "yolox/image_raw");
//        this->declare_parameter<std::string>("publish_boundingbox_topic_name", "yolox/bounding_boxes");
//
//        this->get_parameter("imshow_isshow", this->imshow_);
//        this->get_parameter("model_path", this->model_path_);
//        this->get_parameter("conf", this->conf_th_);
//        this->get_parameter("nms", this->nms_th_);
//        this->get_parameter("device", this->device_);
//        this->get_parameter("model_type", this->model_type_);
//        this->get_parameter("model_version", this->model_version_);
//        this->get_parameter("src_image_topic_name", this->src_image_topic_name_);
//        this->get_parameter("publish_image_topic_name", this->publish_image_topic_name_);
//        this->get_parameter("publish_boundingbox_topic_name", this->publish_boundingbox_topic_name_);
//
//        RCLCPP_INFO(this->get_logger(), "Set parameter imshow_isshow: %i", this->imshow_);
//        RCLCPP_INFO(this->get_logger(), "Set parameter model_path: '%s'", this->model_path_.c_str());
//        RCLCPP_INFO(this->get_logger(), "Set parameter conf: %f", this->conf_th_);
//        RCLCPP_INFO(this->get_logger(), "Set parameter nms: %f", this->nms_th_);
//        RCLCPP_INFO(this->get_logger(), "Set parameter device: %s", this->device_.c_str());
//        RCLCPP_INFO(this->get_logger(), "Set parameter model_type: '%s'", this->model_type_.c_str());
//        RCLCPP_INFO(this->get_logger(), "Set parameter model_version: '%s'", this->model_version_.c_str());
//        RCLCPP_INFO(this->get_logger(), "Set parameter src_image_topic_name: '%s'", this->src_image_topic_name_.c_str());
//        RCLCPP_INFO(this->get_logger(), "Set parameter publish_image_topic_name: '%s'", this->publish_image_topic_name_.c_str());
//
    }
    void YoloXNode::colorImageCallback(const sensor_msgs::Image::ConstPtr &msg) {
        auto img = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = img->image;

        // fps
        auto now = std::chrono::system_clock::now();

        auto objects = this->yolox_->inference(frame);

        auto end = std::chrono::system_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
        ROS_INFO("Inference: %f FPS", 1000.0f / elapsed.count());

        yolox_cpp::utils::draw_objects(frame, objects);
        if (this->imshow_) {
            cv::imshow(this->WINDOW_NAME_, frame);
            auto key = cv::waitKey(1);
            if (key == 27) {
                ros::shutdown();
            }
        }

        auto boxes = objects_to_bboxes(frame, objects, img->header);
        this->pub_bboxes.publish(boxes);

        sensor_msgs::Image::ConstPtr pub_img;
        pub_img = cv_bridge::CvImage(img->header, "bgr8", frame).toImageMsg();
        this->pub_image_.publish(pub_img);

    }

    bboxes_ex_msgs::BoundingBoxes
    YoloXNode::objects_to_bboxes(cv::Mat frame, std::vector<yolox_cpp::Object> objects, std_msgs::Header header) {
        bboxes_ex_msgs::BoundingBoxes boxes;
        boxes.header = header;
        for (auto obj: objects) {
            bboxes_ex_msgs::BoundingBox box;
            box.probability = obj.prob;
            box.class_id = yolox_cpp::COCO_CLASSES[obj.label];
            box.xmin = obj.rect.x;
            box.ymin = obj.rect.y;
            box.xmax = (obj.rect.x + obj.rect.width);
            box.ymax = (obj.rect.y + obj.rect.height);
            box.img_width = frame.cols;
            box.img_height = frame.rows;
            // tracking id
            // box.id = 0;
            // depth
            // box.center_dist = 0;
            boxes.bounding_boxes.emplace_back(box);
        }
        return boxes;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv,"yolox_ros_cpp");
//  ros::NodeOptions node_options;
    yolox_ros_cpp::YoloXNode Node("yolox_node");
//  node_options.enable_topic_statistics(true);
    while (ros::ok()) {
        ros::spin();
    }

//  ros::shutdown();
    return 0;
}


