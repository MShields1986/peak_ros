#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "peak_ros/srv/stream_data.hpp"


namespace reconstruction_namespace {

class ReconstructionComponent : public rclcpp::Node
{
public:
    explicit                                ReconstructionComponent(const rclcpp::NodeOptions& options);

private:
    void                                    initialisePointcloud();
    void                                    callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void                                    publishSrvCb(const std::shared_ptr<peak_ros::srv::StreamData::Request> request,
                                                         std::shared_ptr<peak_ros::srv::StreamData::Response> response);
    void                                    timerCb();

    int32_t                                 rate_;
    std::string                             node_name_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

    rclcpp::Service<peak_ros::srv::StreamData>::SharedPtr          publish_service_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    publisher_;

    rclcpp::TimerBase::SharedPtr            timer_;

    sensor_msgs::msg::PointCloud2                            point_cloud_;
    sensor_msgs::msg::PointCloud2                            output_pointcloud2_;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> buffer_;

    bool                                    use_tf_;
    uint32_t                                b_scan_count_;
    int                                     direction_;
    std::string                             recon_frame_id_;
    bool                                    live_publish_;
    double                                  recon_const_vel_;
    bool                                    flip_direction_;
    rclcpp::Time                            prev_observation_time_{0, 0, RCL_ROS_TIME};

    // TF
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped        trans_;
};

} // namespace reconstruction_namespace
