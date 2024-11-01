#include "reconstruction_nodelet.h"



namespace reconstruction_namespace {

ReconstructionNodelet::ReconstructionNodelet()
 :  rate_(5),
    ns_("/peak"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    b_scan_count_(0),
    direction_(1),
    tfListener_(tfBuffer_)
{
}


void ReconstructionNodelet::onInit()
{
    ros::NodeHandle &nh_ = getMTNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();

    subscriber_ = nh_.subscribe("input", 100, &ReconstructionNodelet::callback, this);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 10, true);
    publish_service_ = nh_.advertiseService(ns_ + "/publish_volume", &ReconstructionNodelet::publishSrvCb, this);
    
    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/process_rate", rate_);
    timer_ = nh_.createTimer(ros::Duration(1 / rate_), &ReconstructionNodelet::timerCb, this);

    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/use_tf", use_tf_);
    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/recon_frame_id", recon_frame_id_);
    ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/live_publish", live_publish_);
    if (!use_tf_) {
        ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/recon_const_vel", recon_const_vel_);
        ReconstructionNodelet::paramHandler(ns_ + "/settings/reconstruction/flip_direction", flip_direction_);
    }

    initialisePointcloud();
}


template <typename ParamType>
ParamType ReconstructionNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << 
        ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


void ReconstructionNodelet::initialisePointcloud() {
    point_cloud_.data.clear();

    int fields          = 5;
    int bytes_per_field = 4;

    point_cloud_.header.stamp = ros::Time::now();
    point_cloud_.header.frame_id = recon_frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_);
    modifier.setPointCloud2Fields(
        fields,
        "x",              1, sensor_msgs::PointField::FLOAT32,   // 32 bits = 4 bytes
        "y",              1, sensor_msgs::PointField::FLOAT32,
        "z",              1, sensor_msgs::PointField::FLOAT32,
        "Amplitudes",     1, sensor_msgs::PointField::FLOAT32,
        "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32
        );

    point_cloud_.height = 1;
    point_cloud_.width = 0;
    point_cloud_.is_dense = true;
    point_cloud_.point_step = fields * bytes_per_field;
    point_cloud_.row_step = point_cloud_.point_step * point_cloud_.width;
    point_cloud_.data.resize(point_cloud_.row_step);
}


void ReconstructionNodelet::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    buffer_.push_back(*msg);
}


bool ReconstructionNodelet::publishSrvCb(peak_ros::StreamData::Request& request,
                                         peak_ros::StreamData::Response& response) {
    NODELET_INFO_STREAM(node_name_ << 
        ": Publish UT volume request received: " << request.stream_data);

    if (request.stream_data) {
        publisher_.publish(point_cloud_);
        NODELET_INFO_STREAM(node_name_ << ": Published reconstruction");
        response.success = true;
        return true;
    } else {
        response.success = true;
        return true;
    }
}


void ReconstructionNodelet::timerCb(const ros::TimerEvent& /*event*/) {
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");

    if (!buffer_.empty()) {
        sensor_msgs::PointCloud2* msg = &buffer_.front();
        sensor_msgs::PointCloud2 output_pointcloud2;

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Live full 3D reconstruction
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (use_tf_) {
            try {
                trans_ = tfBuffer_.lookupTransform(recon_frame_id_,      // target frame
                                                   msg->header.stamp,    // target time
                                                   msg->header.frame_id, // source frame
                                                   msg->header.stamp,    // source time
                                                   // "map",                // fixed frame
                                                   recon_frame_id_,      // fixed frame
                                                   ros::Duration(3.0)    // time out
                                                   );

                tf2::doTransform<sensor_msgs::PointCloud2>(*msg, output_pointcloud2, trans_);

                point_cloud_.width += output_pointcloud2.width;
                uint64_t prev_size = point_cloud_.data.size();
                point_cloud_.data.resize(point_cloud_.data.size() + output_pointcloud2.data.size());

                std::copy(
                    output_pointcloud2.data.begin(),
                    output_pointcloud2.data.end(),
                    point_cloud_.data.begin() + prev_size);

                point_cloud_.header.stamp = msg->header.stamp;

                buffer_.pop_front();

            } catch (tf2::TransformException& ex) {
                NODELET_WARN_STREAM(node_name_ << 
                    ": Could not find transform " << recon_frame_id_ << 
                    " to " << msg->header.frame_id << 
                    ": " << ex.what());
            }

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Post process psuedo 3D reconstruction for plotting
        ////////////////////////////////////////////////////////////////////////////////////////////
        } else {
            trans_.header.stamp = msg->header.stamp;
            trans_.header.frame_id = recon_frame_id_;
            trans_.child_frame_id = msg->header.frame_id;


            if (b_scan_count_ == 0) {
                trans_.transform.translation.x = 0.0l;
                // trans_.transform.translation.x = 0.0751l;
                trans_.transform.translation.y = 0.0l;
                trans_.transform.translation.z = 0.0l;
                trans_.transform.rotation.x =  0.0l;
                trans_.transform.rotation.y = -1.0l;
                trans_.transform.rotation.z =  0.0l;
                trans_.transform.rotation.w =  0.0l;

            } else {
                ros::Duration dt = msg->header.stamp - prev_observation_time_;

                // During scan pass
                if (dt.toSec() < 4.0l) {
                    if (direction_ == 1) {
                        NODELET_INFO_STREAM_THROTTLE(30, node_name_ << ": Going forwards");
                        trans_.transform.translation.x += recon_const_vel_ * dt.toSec();
                    } else {
                        NODELET_INFO_STREAM_THROTTLE(30, node_name_ << ": Going backwards");
                        trans_.transform.translation.x -= recon_const_vel_ * dt.toSec();
                    }
                // Switching raster paths
                } else {
                    publisher_.publish(point_cloud_);
                    NODELET_INFO_STREAM(node_name_ << ": Published reconstruction");
                    // initialisePointcloud();
                    if (flip_direction_) {
                        if (direction_ == 1) {
                            NODELET_INFO_STREAM(node_name_ << ": Changing direction");
                            direction_ = -1;
                            trans_.transform.translation.y += 0.09l;
                            trans_.transform.rotation.x =  1.0l;
                            trans_.transform.rotation.y =  0.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        } else {
                            NODELET_INFO_STREAM(node_name_ << ": Changing direction");
                            direction_ = 1;
                            trans_.transform.rotation.x =  0.0l;
                            trans_.transform.rotation.y = -1.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        }
                    } else {
                        trans_.transform.translation.x  = 0.0l;
                        trans_.transform.translation.y += 0.045l;
                        NODELET_INFO_STREAM(node_name_ << ": Reset start of pass to zero: " << trans_.transform.translation.x);
                    }
                }
            }

            tf2::doTransform<sensor_msgs::PointCloud2>(*msg, output_pointcloud2, trans_);

            // NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": transform translation x: " << trans_.transform.translation.x);
            // NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": transform translation y: " << trans_.transform.translation.y);
            // NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": transform translation z: " << trans_.transform.translation.z);

            point_cloud_.width += output_pointcloud2.width;
            uint64_t prev_size = point_cloud_.data.size();
            point_cloud_.data.resize(point_cloud_.data.size() + output_pointcloud2.data.size());

            std::copy(
                output_pointcloud2.data.begin(),
                output_pointcloud2.data.end(),
                point_cloud_.data.begin() + prev_size);

            point_cloud_.header.stamp = msg->header.stamp;

            buffer_.pop_front();

            prev_observation_time_ = msg->header.stamp;
            b_scan_count_++;
        }

        if (live_publish_) {
            publisher_.publish(point_cloud_);
        }
    }
}


} // namespace reconstruction_namespace

PLUGINLIB_EXPORT_CLASS(reconstruction_namespace::ReconstructionNodelet, nodelet::Nodelet);