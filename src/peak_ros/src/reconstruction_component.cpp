#include "reconstruction_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>


namespace reconstruction_namespace {

ReconstructionComponent::ReconstructionComponent(const rclcpp::NodeOptions& options)
 :  rclcpp::Node("reconstruction_node", options),
    rate_(5),
    b_scan_count_(0),
    direction_(1)
{
    node_name_ = get_name();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", rclcpp::QoS(10),
        std::bind(&ReconstructionComponent::callback, this, std::placeholders::_1));
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "output", rclcpp::QoS(10).transient_local());
    publish_service_ = create_service<peak_ros::srv::StreamData>(
        "publish_volume",
        std::bind(&ReconstructionComponent::publishSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    rate_ = declare_parameter<int>("settings.reconstruction.process_rate", 5);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / static_cast<double>(rate_)),
        std::bind(&ReconstructionComponent::timerCb, this));

    use_tf_         = declare_parameter<bool>("settings.reconstruction.use_tf", true);
    recon_frame_id_ = declare_parameter<std::string>("settings.reconstruction.recon_frame_id", "map");
    live_publish_   = declare_parameter<bool>("settings.reconstruction.live_publish", true);
    if (!use_tf_) {
        recon_const_vel_ = declare_parameter<double>("settings.reconstruction.recon_const_vel", 0.05);
        flip_direction_  = declare_parameter<bool>("settings.reconstruction.flip_direction", true);
    }

    initialisePointcloud();
}


void ReconstructionComponent::initialisePointcloud() {
    point_cloud_.data.clear();

    int fields          = 5;
    int bytes_per_field = 4;

    point_cloud_.header.stamp = this->now();
    point_cloud_.header.frame_id = recon_frame_id_;
    sensor_msgs::PointCloud2Modifier modifier(point_cloud_);
    modifier.setPointCloud2Fields(
        fields,
        "x",              1, sensor_msgs::msg::PointField::FLOAT32,   // 32 bits = 4 bytes
        "y",              1, sensor_msgs::msg::PointField::FLOAT32,
        "z",              1, sensor_msgs::msg::PointField::FLOAT32,
        "Amplitudes",     1, sensor_msgs::msg::PointField::FLOAT32,
        "TimeofFlight", 1, sensor_msgs::msg::PointField::FLOAT32
        );

    point_cloud_.height = 1;
    point_cloud_.width = 0;
    point_cloud_.is_dense = true;
    point_cloud_.point_step = fields * bytes_per_field;
    point_cloud_.row_step = point_cloud_.point_step * point_cloud_.width;
    point_cloud_.data.resize(point_cloud_.row_step);
}


void ReconstructionComponent::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    buffer_.push_back(msg);
}


void ReconstructionComponent::publishSrvCb(const std::shared_ptr<peak_ros::srv::StreamData::Request> request,
                                           std::shared_ptr<peak_ros::srv::StreamData::Response> response) {
    RCLCPP_INFO_STREAM(get_logger(), node_name_ <<
        ": Publish UT volume request received: " << request->stream_data);

    if (request->stream_data) {
        publisher_->publish(point_cloud_);
        RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Published reconstruction");
        response->success = true;
    } else {
        response->success = true;
    }
}


void ReconstructionComponent::timerCb() {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 600000, node_name_ << ": Node running");

    if (!buffer_.empty()) {
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg = buffer_.front();

        ////////////////////////////////////////////////////////////////////////////////////////////
        // Live full 3D reconstruction
        ////////////////////////////////////////////////////////////////////////////////////////////
        if (use_tf_) {
            try {
                trans_ = tf_buffer_->lookupTransform(recon_frame_id_,                       // target frame
                                                     rclcpp::Time(msg->header.stamp),        // target time
                                                     msg->header.frame_id,                   // source frame
                                                     rclcpp::Time(msg->header.stamp),        // source time
                                                     // "map",                               // fixed frame
                                                     recon_frame_id_,                        // fixed frame
                                                     rclcpp::Duration::from_seconds(3.0)     // time out
                                                     );

                tf2::doTransform<sensor_msgs::msg::PointCloud2>(*msg, output_pointcloud2_, trans_);

                point_cloud_.width += output_pointcloud2_.width;
                point_cloud_.data.insert(point_cloud_.data.end(),
                    output_pointcloud2_.data.begin(),
                    output_pointcloud2_.data.end());

                point_cloud_.header.stamp = msg->header.stamp;

                buffer_.pop_front();

            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_STREAM(get_logger(), node_name_ <<
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
                rclcpp::Duration dt = rclcpp::Time(msg->header.stamp) - prev_observation_time_;

                // During scan pass
                if (dt.seconds() < 4.0l) {
                    if (direction_ == 1) {
                        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 30000, node_name_ << ": Going forwards");
                        trans_.transform.translation.x += recon_const_vel_ * dt.seconds();
                    } else {
                        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 30000, node_name_ << ": Going backwards");
                        trans_.transform.translation.x -= recon_const_vel_ * dt.seconds();
                    }
                // Switching raster paths
                } else {
                    publisher_->publish(point_cloud_);
                    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Published reconstruction");
                    // initialisePointcloud();
                    if (flip_direction_) {
                        if (direction_ == 1) {
                            RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Changing direction");
                            direction_ = -1;
                            trans_.transform.translation.y += 0.09l;
                            trans_.transform.rotation.x =  1.0l;
                            trans_.transform.rotation.y =  0.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        } else {
                            RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Changing direction");
                            direction_ = 1;
                            trans_.transform.rotation.x =  0.0l;
                            trans_.transform.rotation.y = -1.0l;
                            trans_.transform.rotation.z =  0.0l;
                            trans_.transform.rotation.w =  0.0l;
                        }
                    } else {
                        trans_.transform.translation.x  = 0.0l;
                        trans_.transform.translation.y += 0.045l;
                        RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Reset start of pass to zero: " << trans_.transform.translation.x);
                    }
                }
            }

            tf2::doTransform<sensor_msgs::msg::PointCloud2>(*msg, output_pointcloud2_, trans_);

            // RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, node_name_ << ": transform translation x: " << trans_.transform.translation.x);
            // RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, node_name_ << ": transform translation y: " << trans_.transform.translation.y);
            // RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 10000, node_name_ << ": transform translation z: " << trans_.transform.translation.z);

            point_cloud_.width += output_pointcloud2_.width;
            point_cloud_.data.insert(point_cloud_.data.end(),
                output_pointcloud2_.data.begin(),
                output_pointcloud2_.data.end());

            point_cloud_.header.stamp = msg->header.stamp;

            buffer_.pop_front();

            prev_observation_time_ = rclcpp::Time(msg->header.stamp);
            b_scan_count_++;
        }

        if (live_publish_) {
            publisher_->publish(point_cloud_);
        }
    }
}


} // namespace reconstruction_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(reconstruction_namespace::ReconstructionComponent)
