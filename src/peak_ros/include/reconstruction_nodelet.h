#pragma once

#include <signal.h>
#include <algorithm>
#include <deque>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "peak_ros/StreamData.h"


namespace reconstruction_namespace {

class ReconstructionNodelet : public nodelet::Nodelet
{
public:
    explicit                                ReconstructionNodelet();

private:
    virtual void                            onInit();
    template <typename ParamType>
    ParamType                               paramHandler(std::string param_name, ParamType& param_value);
    void                                    initialisePointcloud();
    void                                    callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool                                    publishSrvCb(peak_ros::StreamData::Request& request,
                                                         peak_ros::StreamData::Response& response);
    void                                    timerCb(const ros::TimerEvent& event);

    ros::NodeHandle                         nh_;
    int32_t                                 rate_;
    std::string                             node_name_;
    std::string                             ns_;

    ros::Subscriber                         subscriber_;

    ros::ServiceServer                      publish_service_;
    ros::Publisher                          publisher_;

    ros::Timer                              timer_;

    sensor_msgs::PointCloud2                point_cloud_;
    std::deque<sensor_msgs::PointCloud2>    buffer_;

    bool                                    use_tf_;
    uint32_t                                b_scan_count_;
    int                                     direction_;
    std::string                             recon_frame_id_;
    bool                                    live_publish_;
    double                                  recon_const_vel_;
    bool                                    flip_direction_;
    ros::Time                               prev_observation_time_;

    // TF
    tf2_ros::Buffer                         tfBuffer_;
    tf2_ros::TransformListener              tfListener_;
    geometry_msgs::TransformStamped         trans_;
};

} // namespace reconstruction_namespace