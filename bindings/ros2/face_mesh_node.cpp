/*
 *  Copyright 2024 (C) Jeroen Veen <ducroq> & Victor Hogeweij <Hoog-V>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * This file is part of the Face_Mesh.Cpp library
 *
 * Author:         Jeroen Veen <ducroq>
 *                 Victor Hogeweij <hoog-v>
 *
 */
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "face_mesh.hpp"

class FaceMeshNode : public rclcpp::Node
{
public:
    FaceMeshNode() : Node("face_mesh_node")
    {
        declare_and_get_parameters();
        det_.load_model(CLFML_FACE_MESH_CPU_MODEL_PATH);
        setup_communication();
        detection_enabled_ = true;
    }

private:
    void declare_and_get_parameters()
    {
        declare_parameter("roi_topic", "/face_roi");
        declare_parameter("camera_topic", "/image_raw");
        declare_parameter("face_mesh_landmarks_topic", "/face_mesh_landmarks");

        camera_topic_ = get_parameter("camera_topic").as_string();
        face_landmarks_topic_ = get_parameter("face_mesh_landmarks_topic").as_string();
        roi_topic_ = get_parameter("roi_topic").as_string();
    }

    void setup_communication()
    {
        // Subscribe to the on-demand trigger topic
        on_demand_sub_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
            roi_topic_, 10,
            std::bind(&FaceMeshNode::on_demand_callback, this, std::placeholders::_1));

        // Subscribe to the image topic but only store the latest image
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            camera_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FaceMeshNode::store_latest_image, this, std::placeholders::_1));

        face_mesh_landmarks_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(face_landmarks_topic_, 10);

        toggle_service_ = create_service<std_srvs::srv::SetBool>(
            "toggle_face_mesh",
            std::bind(&FaceMeshNode::toggle_detection, this, std::placeholders::_1, std::placeholders::_2));
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!detection_enabled_)
            return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        cv::Mat cropped_image_to_roi = cv_ptr->image(roi_);
        det_.load_image(cropped_image_to_roi, roi_);

        publish_face_mesh_landmarks();
    }
    void store_latest_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        last_image_msg_ = msg; // Store the latest image in memory
    }

    void on_demand_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        if (!detection_enabled_ || msg->width == 0 || msg->height == 0)
            return;

        if (last_image_msg_)
        {
            roi_ = cv::Rect(msg->x_offset, msg->y_offset, msg->width, msg->height);
            if (roi_.x >= 0 && roi_.y >= 0 &&
                roi_.x + roi_.width <= last_image_msg_->width &&
                roi_.y + roi_.height <= last_image_msg_->height)
            {
                image_callback(last_image_msg_); // Process the last stored image
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(), "No image available when trigger received.");
        }
    }

    void publish_face_mesh_landmarks()
    {
        std::array<cv::Point3f, CLFML::FaceMesh::NUM_OF_FACE_MESH_POINTS> face_landmarks = det_.get_face_mesh_points();
        if (face_landmarks.empty())
        {
            RCLCPP_WARN(get_logger(), "No face landmarks detected.");
            return;
        }

        // Create PointCloud2 message
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = now();
        cloud_msg.header.frame_id = "camera_frame"; // Set the reference frame
        cloud_msg.height = 1;                       // Single row
        cloud_msg.width = face_landmarks.size();    // Number of points
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        // Define the fields for x, y, z
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3,
                                      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32);

        modifier.resize(face_landmarks.size()); // Resize for the number of points

        // Create iterators for point data
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        // Fill PointCloud2 data
        for (const auto &landmark : face_landmarks)
        {
            *iter_x = landmark.x;
            *iter_y = landmark.y;
            *iter_z = landmark.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        // Publish the PointCloud2 message
        face_mesh_landmarks_pub_->publish(cloud_msg);
    }

    void toggle_detection(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        detection_enabled_ = request->data;
        response->success = true;
        response->message = detection_enabled_ ? "Face mesh enabled" : "Face mesh disabled";
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr on_demand_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr face_mesh_landmarks_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;

    sensor_msgs::msg::Image::SharedPtr last_image_msg_;

    CLFML::FaceMesh::FaceMesh det_;
    std::string camera_topic_;
    std::string face_landmarks_topic_;
    std::string roi_topic_;
    cv::Rect roi_ = cv::Rect(0, 0, 0, 0);
    bool detection_enabled_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaceMeshNode>());
    rclcpp::shutdown();
    return 0;
}