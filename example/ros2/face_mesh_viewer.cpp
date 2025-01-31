#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <opencv2/opencv.hpp>

class FaceMeshViewer : public rclcpp::Node
{
public:
    FaceMeshViewer() : Node("face_mesh_viewer")
    {
        // Declare parameters for topic names
        this->declare_parameter("camera_topic", "/image_raw");
        this->declare_parameter("face_landmarks_topic", "/face_mesh_landmarks");

        // Get parameter values
        std::string camera_topic = this->get_parameter("camera_topic").as_string();
        std::string face_landmarks_topic = this->get_parameter("face_landmarks_topic").as_string();

        // Create subscriptions
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic, 10,
            std::bind(&FaceMeshViewer::image_callback, this, std::placeholders::_1));

        face_landmarks_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            face_landmarks_topic, 10,
            std::bind(&FaceMeshViewer::face_landmarks_callback, this, std::placeholders::_1));

        // Create OpenCV window
        cv::namedWindow("Face Mesh Viewer", cv::WINDOW_NORMAL);
        cv::resizeWindow("Face Mesh Viewer", 800, 600);
    }

    ~FaceMeshViewer()
    {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            current_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            update_display();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void face_landmarks_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->width * msg->height != face_landmarks_.size())
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected face landmarks size: %d", msg->width * msg->height);
            return;
        }

        // Create iterators for PointCloud2
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        // Extract landmark points
        for (size_t i = 0; i < face_landmarks_.size(); ++i, ++iter_x, ++iter_y, ++iter_z)
        {
            face_landmarks_[i] = cv::Point3f(*iter_x, *iter_y, *iter_z);
        }

        update_display();
    }

    void update_display()
    {
        if (current_frame_.empty())
            return;

        cv::Mat display_frame = current_frame_.clone();

        // Draw face landmarks
        for (cv::Point3f keypoint : face_landmarks_)
        {
            cv::circle(display_frame, cv::Point(keypoint.x, keypoint.y), 2, cv::Scalar(0, 255, 0), -1);
        }

        // Display the frame
        cv::imshow("Face Mesh Viewer", display_frame);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr face_landmarks_sub_;

    cv::Mat current_frame_;
    std::array<cv::Point3f, 468> face_landmarks_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceMeshViewer>());
    rclcpp::shutdown();
    return 0;
}