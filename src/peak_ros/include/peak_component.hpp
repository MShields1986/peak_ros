#pragma once

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "PeakMicroPulseHandler/peak_handler.h"

#include "peak_ros/msg/ascan.hpp"
#include "peak_ros/msg/observation.hpp"

#include <std_srvs/srv/trigger.hpp>

#include "peak_ros/srv/stream_data.hpp"


namespace peak_namespace {

class PeakComponent : public rclcpp::Node {
public:
    explicit                           PeakComponent(const rclcpp::NodeOptions& options);

private:
    void                               initHardware();
    void                               prePopulateAScanMessage();
    void                               prePopulateBScanMessage();
    void                               prePopulateGatedBScanMessage();
    void                               streamDataSrvCb(const std::shared_ptr<peak_ros::srv::StreamData::Request> request,
                                                       std::shared_ptr<peak_ros::srv::StreamData::Response> response);
    void                               takeMeasurementSrvCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void                               takeMeasurement();
    void                               populateAScanMessage();
    void                               populateBScanMessage(const peak_ros::msg::Observation& obs_msg);
    void                               timerCb();

    std::string                        node_name_;
    int                                digitisation_rate_;
    bool                               profile_;

    // Config
    int                                acquisition_rate_;
    std::string                        peak_address_;
    int                                peak_port_;
    std::string                        mps_file_;
    std::string                        package_path_;

    // TCG
    bool                               use_tcg_;
    float                              amp_factor_;
    float                              depth_factor_;
    float                              tcg_limit_;

    // Gates
    float                              gate_front_wall_;
    float                              depth_to_skip_;
    float                              gate_back_wall_;
    float                              max_depth_;
    bool                               zero_to_front_wall_;
    bool                               show_front_wall_;

    // Input
    PeakHandler                        peak_handler_;
    const PeakHandler::OutputFormat*   ltpa_data_ptr_;

    // Output
    peak_ros::msg::Observation         ltpa_msg_;
    sensor_msgs::msg::PointCloud2      bscan_cloud_;
    sensor_msgs::msg::PointCloud2      gated_bscan_cloud_;

    bool                               stream_;

    rclcpp::Publisher<peak_ros::msg::Observation>::SharedPtr      ascan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   bscan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   gated_bscan_publisher_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   single_measure_service_;
    rclcpp::Service<peak_ros::srv::StreamData>::SharedPtr stream_service_;

    rclcpp::TimerBase::SharedPtr       timer_;
};

} // namespace peak_namespace
