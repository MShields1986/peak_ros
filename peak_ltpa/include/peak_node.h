#pragma once

#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
//#include <nodelet/nodelet.h>
//#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "peak_handler.h"

#include "peak_ltpa/Ascan.h"
#include "peak_ltpa/Observation.h"

#include "peak_ltpa/StreamData.h"
#include "peak_ltpa/TakeSingleMeasurement.h"



class PeakNode {
public:
    PeakNode(bool* kill_now);
    //~PeakNode();

private:
    template <typename ParamType>
    ParamType                   paramHandler(std::string param_name, ParamType& param_value);

    void                        initHardware();
    void                        prePopulateMessage();
    bool                        streamDataSrvCb(peak_ltpa::StreamData::Request& request,
                                                peak_ltpa::StreamData::Response& response);
    bool                        takeMeasurementSrvCb(peak_ltpa::TakeSingleMeasurement::Request& request,
                                                     peak_ltpa::TakeSingleMeasurement::Response& response);
    void                        takeMeasurement();
    void                        populateAScanMessage();
    void                        populateBScanMessage(const peak_ltpa::Observation& ascan_msg);

public:
    void                        run();

private:
    ros::NodeHandle             nh_;
    ros::Rate                   rate_;
    bool*                       kill_now_;
    std::string                 node_name_;
    std::string                 ns_;

    std::string                 package_path_;

    int                         acquisition_rate_;
    std::string                 peak_address_;
    int                         peak_port_;
    std::string                 mps_file_;

    PeakHandler                 peak_handler_;

    const PeakHandler::OutputFormat*     ltpa_data_ptr_;

    peak_ltpa::Observation      ltpa_msg_;

    bool                        stream_;

    ros::Publisher              ascan_publisher_;
    ros::Publisher              bscan_publisher_;

    ros::ServiceServer          single_measure_service_;
    ros::ServiceServer          stream_service_;

    //ros::Timer                  timer_;
};