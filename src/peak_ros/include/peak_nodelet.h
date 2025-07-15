#pragma once

#include <cmath>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "PeakMicroPulseHandler/peak_handler.h"

#include "peak_ros/Ascan.h"
#include "peak_ros/Observation.h"

#include "peak_ros/StreamData.h"
#include "peak_ros/TakeSingleMeasurement.h"


namespace peak_namespace {

class PeakNodelet : public nodelet::Nodelet {
public:
    PeakNodelet();
    //~PeakNode();

private:
    virtual void                       onInit();

    template <typename ParamType>
    ParamType                          paramHandler(std::string param_name, ParamType& param_value);

    void                               initHardware();
    void                               prePopulateAScanMessage();
    void                               prePopulateBScanMessage();
    void                               prePopulateGatedBScanMessage();
    bool                               streamDataSrvCb(peak_ros::StreamData::Request& request,
                                                       peak_ros::StreamData::Response& response);
    bool                               takeMeasurementSrvCb(peak_ros::TakeSingleMeasurement::Request& request,
                                                            peak_ros::TakeSingleMeasurement::Response& response);
    void                               takeMeasurement();
    void                               populateAScanMessage();
    void                               populateBScanMessage(const peak_ros::Observation& obs_msg);
    void                               timerCb(const ros::TimerEvent& /*event*/);

    ros::NodeHandle                    nh_;
    ros::Rate                          rate_;
    int                                digitisation_rate_;
    std::string                        node_name_;
    std::string                        ns_;
    std::string                        package_path_;
    bool                               profile_;

    // Config
    int                                acquisition_rate_;
    std::string                        peak_address_;
    int                                peak_port_;
    std::string                        mps_file_;

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
    peak_ros::Observation              ltpa_msg_;
    sensor_msgs::PointCloud2           bscan_cloud_;
    sensor_msgs::PointCloud2           gated_bscan_cloud_;

    bool                               stream_;

    ros::Publisher                     ascan_publisher_;
    ros::Publisher                     bscan_publisher_;
    ros::Publisher                     gated_bscan_publisher_;

    ros::ServiceServer                 single_measure_service_;
    ros::ServiceServer                 stream_service_;

    ros::Timer                         timer_;

};

} // namespace peak_namespace