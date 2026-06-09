#include "peak_component.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace peak_namespace {

PeakComponent::PeakComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("peak_node", options),
    peak_handler_()
{
    node_name_ = get_name();
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Initialising node...");

    std::filesystem::path pkg_share;
    ament_index_cpp::get_package_share_directory("peak_ros", pkg_share);
    package_path_ = pkg_share.string();

    // Connection / acquisition configuration
    acquisition_rate_ = declare_parameter<int>("settings.acquisition_rate", 20);
    peak_address_     = declare_parameter<std::string>("settings.peak_address", "10.1.1.2");
    peak_port_        = declare_parameter<int>("settings.peak_port", 1067);
    mps_file_         = declare_parameter<std::string>("settings.mps_file", "");

    peak_handler_.setup(
        acquisition_rate_,
        peak_address_,
        peak_port_,
        package_path_ + "/mps/" + mps_file_
    );

    digitisation_rate_ = declare_parameter<int>("settings.digitisation_rate", 100);
    profile_           = declare_parameter<bool>("settings.profile", false);

    use_tcg_      = declare_parameter<bool>("settings.tcg.use_tcg", false);
    amp_factor_   = static_cast<float>(declare_parameter<double>("settings.tcg.amp_factor", 1.5));
    depth_factor_ = static_cast<float>(declare_parameter<double>("settings.tcg.depth_factor", 1.5));
    tcg_limit_    = static_cast<float>(declare_parameter<double>("settings.tcg.tcg_limit", 0.7));
    depth_factor_ = depth_factor_ * 0.001f;

    gate_front_wall_    = static_cast<float>(declare_parameter<double>("settings.gates.gate_front_wall", 0.2));
    depth_to_skip_      = static_cast<float>(declare_parameter<double>("settings.gates.depth_to_skip", 6.5));
    gate_back_wall_     = static_cast<float>(declare_parameter<double>("settings.gates.gate_back_wall", 0.7));
    max_depth_          = static_cast<float>(declare_parameter<double>("settings.gates.max_depth", 17.0));
    zero_to_front_wall_ = declare_parameter<bool>("settings.gates.zero_to_front_wall", true);
    show_front_wall_    = declare_parameter<bool>("settings.gates.show_front_wall", false);
    depth_to_skip_ = depth_to_skip_ * 0.001f;
    max_depth_ = max_depth_ * 0.001f;

    initHardware();

    prePopulateAScanMessage();
    precomputeBScanLookups();
    prePopulateBScanMessage();
    prePopulateGatedBScanMessage();

    // Streaming sensor data: small, non-latched queue. Latched (transient_local)
    // QoS is avoided here as it is incompatible with intra-process comms.
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(3));

    ascan_publisher_       = create_publisher<peak_ros::msg::Observation>("a_scans", qos);
    bscan_publisher_       = create_publisher<sensor_msgs::msg::PointCloud2>("b_scan", qos);
    gated_bscan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("gated_b_scan", qos);

    single_measure_service_ = create_service<std_srvs::srv::Trigger>(
        "take_single_measurement",
        std::bind(&PeakComponent::takeMeasurementSrvCb, this, std::placeholders::_1, std::placeholders::_2));
    stream_service_ = create_service<peak_ros::srv::StreamData>(
        "stream_data",
        std::bind(&PeakComponent::streamDataSrvCb, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / static_cast<double>(acquisition_rate_)),
        std::bind(&PeakComponent::timerCb, this));

    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Node initialised");
}


PeakComponent::~PeakComponent() {
    // Stop the asynchronous acquisition (cancels the in-flight async read,
    // joins the IO thread and drains the socket) so shutdown never blocks
    // waiting on the LTPA mid-packet.
    peak_handler_.stopAsyncAcquisition();
}


void PeakComponent::initHardware() {
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Initialising Peak hardware...");

    peak_handler_.connect();

    int reset_sleep = declare_parameter<int>("settings.reset_sleep_seconds", 10);
    peak_handler_.sendReset(digitisation_rate_, reset_sleep);

    peak_handler_.readMpsFile();
    peak_handler_.sendMpsConfiguration();

    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Peak hardware initialised");
}


void PeakComponent::prePopulateAScanMessage() {
    ltpa_msg_.header.frame_id = declare_parameter<std::string>("settings.frame_id", "ltpa");

    // Get settings the PeakHandler extracted from the .mps file
    ltpa_msg_.dof = peak_handler_.dof_;
    ltpa_msg_.gate_start = peak_handler_.gate_start_;
    ltpa_msg_.gate_end = peak_handler_.gate_end_;
    ltpa_msg_.ascan_length = peak_handler_.ascan_length_;
    ltpa_msg_.num_ascans = peak_handler_.num_a_scans_;
    ltpa_msg_.ascans.reserve(ltpa_msg_.num_ascans);

    // digitisation_rate is set during sendReset; read it from the handler's data pointer
    ltpa_msg_.digitisation_rate = peak_handler_.ltpa_data_ptr()->digitisation_rate;

    // TODO: Consider sending this as a separate one time latched message rather than repeated here
    ltpa_msg_.n_elements           = declare_parameter<int>("settings.boundary_conditions.n_elements", 0);
    ltpa_msg_.element_pitch         = declare_parameter<double>("settings.boundary_conditions.element_pitch", 0.0);
    ltpa_msg_.inter_element_spacing = declare_parameter<double>("settings.boundary_conditions.inter_element_spacing", 0.0);
    ltpa_msg_.element_width         = declare_parameter<double>("settings.boundary_conditions.element_width", 0.0);
    ltpa_msg_.vel_wedge             = declare_parameter<double>("settings.boundary_conditions.vel_wedge", 0.0);
    ltpa_msg_.vel_couplant          = declare_parameter<double>("settings.boundary_conditions.vel_couplant", 0.0);
    ltpa_msg_.vel_material          = declare_parameter<double>("settings.boundary_conditions.vel_material", 0.0);
    ltpa_msg_.wedge_angle           = declare_parameter<double>("settings.boundary_conditions.wedge_angle", 0.0);
    ltpa_msg_.wedge_depth           = declare_parameter<double>("settings.boundary_conditions.wedge_depth", 0.0);
    ltpa_msg_.couplant_depth        = declare_parameter<double>("settings.boundary_conditions.couplant_depth", 0.0);
    ltpa_msg_.specimen_depth        = declare_parameter<double>("settings.boundary_conditions.specimen_depth", 0.0);

    // Not entirely necessary as implemented here but we can pass these back to the peak_handler
    peak_handler_.setReconstructionConfiguration(
        ltpa_msg_.n_elements,
        ltpa_msg_.element_pitch,
        ltpa_msg_.inter_element_spacing,
        ltpa_msg_.element_width,
        ltpa_msg_.vel_wedge,
        ltpa_msg_.vel_couplant,
        ltpa_msg_.vel_material,
        ltpa_msg_.wedge_angle,
        ltpa_msg_.wedge_depth,
        ltpa_msg_.couplant_depth,
        ltpa_msg_.specimen_depth
        );
}


void PeakComponent::precomputeBScanLookups() {
    int ascan_len = ltpa_msg_.ascan_length;
    int num_elements = ltpa_msg_.num_ascans;
    float dt = 1.0f / ((float)ltpa_msg_.digitisation_rate * 1000000.0f);
    float vel_material = (float)ltpa_msg_.vel_material;
    float element_pitch = (float)ltpa_msg_.element_pitch * 0.001f; // mm to m

    // Z lookup: depth for each sample index
    z_lookup_.resize(ascan_len);
    for (int i = 0; i < ascan_len; ++i) {
        z_lookup_[i] = (float)i * vel_material * dt / 2.0f;
    }

    // Y lookup: y position for each element
    y_lookup_.resize(num_elements);
    for (int e = 0; e < num_elements; ++e) {
        y_lookup_[e] = (float)e * element_pitch;
    }

    // TCG gain lookup: precompute pow() for each sample
    tcg_gain_.resize(ascan_len);
    for (int i = 0; i < ascan_len; ++i) {
        float z = z_lookup_[i];
        if (use_tcg_ && z > (10.0f * 0.001f)) {
            tcg_gain_[i] = std::pow(10.0f, (amp_factor_ * (z / depth_factor_) / 20.0f));
        } else {
            tcg_gain_[i] = 1.0f;
        }
    }

    lookups_valid_ = true;
}


void PeakComponent::prePopulateBScanMessage() {
    int fields          = 4;
    int bytes_per_field = 4; // 32 bits = 4 bytes

    sensor_msgs::PointCloud2Modifier bscan_cloud_modifier(bscan_cloud_);
    bscan_cloud_modifier.setPointCloud2Fields(
        fields,
        "x", 1,          sensor_msgs::msg::PointField::FLOAT32,
        "y", 1,          sensor_msgs::msg::PointField::FLOAT32,
        "z", 1,          sensor_msgs::msg::PointField::FLOAT32,
        "Amplitudes", 1, sensor_msgs::msg::PointField::FLOAT32
        );
    bscan_cloud_.height = 1;
    bscan_cloud_.is_dense = true;
    bscan_cloud_.point_step = fields * bytes_per_field;
}


void PeakComponent::prePopulateGatedBScanMessage() {
    int fields          = 5;
    int bytes_per_field = 4; // 32 bits = 4 bytes

    sensor_msgs::PointCloud2Modifier gated_bscan_cloud_modifier(gated_bscan_cloud_);
    gated_bscan_cloud_modifier.setPointCloud2Fields(
        fields,
        "x", 1,            sensor_msgs::msg::PointField::FLOAT32,
        "y", 1,            sensor_msgs::msg::PointField::FLOAT32,
        "z", 1,            sensor_msgs::msg::PointField::FLOAT32,
        "Amplitudes", 1,   sensor_msgs::msg::PointField::FLOAT32,
        "TimeofFlight", 1, sensor_msgs::msg::PointField::FLOAT32
        );
    gated_bscan_cloud_.height = 1;
    gated_bscan_cloud_.is_dense = true;
    gated_bscan_cloud_.point_step = fields * bytes_per_field;
}


void PeakComponent::streamDataSrvCb(const std::shared_ptr<peak_ros::srv::StreamData::Request> request,
                                    std::shared_ptr<peak_ros::srv::StreamData::Response> response) {
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Streaming request received: " << request->stream_data);
    if (request->stream_data) {
        stream_ = true;
        // Cancellable, non-blocking acquisition driven by the handler's IO thread.
        peak_handler_.startAsyncAcquisition(nullptr, acquisition_rate_);
        response->success = true;
    } else {
        stream_ = false;
        peak_handler_.stopAsyncAcquisition();
        response->success = true;
    }
}


void PeakComponent::takeMeasurementSrvCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Take single measurement request received");

    // A single (synchronous) measurement and asynchronous streaming both drive
    // the same socket, so they are mutually exclusive.
    if (stream_) {
        response->success = false;
        response->message = "Streaming is active; call stream_data false before a single measurement";
        return;
    }

    takeMeasurement();
    response->success = true;
    response->message = "Single measurement taken";
}


void PeakComponent::takeMeasurement() {
    std::lock_guard<std::mutex> lock(processing_mutex_);

    // TODO: Remove profiling when happy with acquisition rates
    std::chrono::high_resolution_clock::time_point begin;
    std::chrono::high_resolution_clock::time_point end;
    if (profile_) begin = std::chrono::high_resolution_clock::now();

    if (peak_handler_.sendDataRequest()) {
        const auto* data_ptr = peak_handler_.ltpa_data_ptr();
        if (data_ptr) {
            latest_data_ = *data_ptr;
        }
        processMeasurement();
    }

    if (profile_) {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [PeakComponent::takeMeasurement()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }
}


void PeakComponent::processMeasurement() {
    std::chrono::high_resolution_clock::time_point begin;
    std::chrono::high_resolution_clock::time_point end_1;
    std::chrono::high_resolution_clock::time_point end_2;
    std::chrono::high_resolution_clock::time_point end_3;
    std::chrono::high_resolution_clock::time_point end_4;

    if (profile_) begin = std::chrono::high_resolution_clock::now();

    populateAScanMessage();

    if (profile_) {
        end_1 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [PeakComponent::populateAScanMessage()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_1-begin).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    ascan_publisher_->publish(ltpa_msg_);

    if (profile_) {
        end_2 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [ascan_publisher_->publish(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_2-end_1).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    populateBScanMessage(ltpa_msg_);

    if (profile_) {
        end_3 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [PeakComponent::populateBScanMessage(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_3-end_2).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    bscan_publisher_->publish(bscan_cloud_);
    gated_bscan_publisher_->publish(gated_bscan_cloud_);

    if (profile_) {
        end_4 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [publish bscan + gated] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_4-end_3).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }
}


void PeakComponent::populateAScanMessage() {
    ltpa_msg_.header.stamp = this->now();
    ltpa_msg_.ascans.clear();

    for (auto& ascan : latest_data_.ascans) {
        peak_ros::msg::Ascan ascan_msg;
        ascan_msg.count = ascan.header.count;
        ascan_msg.test_number = ascan.header.testNo;
        ascan_msg.dof = ascan.header.dof;
        ascan_msg.channel = ascan.header.channel;
        ascan_msg.amplitudes = std::move(ascan.amps);
        ltpa_msg_.ascans.push_back(std::move(ascan_msg));
    }

    ltpa_msg_.max_amplitude = latest_data_.max_amplitude;
}


void PeakComponent::populateBScanMessage(const peak_ros::msg::Observation& obs_msg) {
    bscan_cloud_.header.stamp = obs_msg.header.stamp;
    bscan_cloud_.header.frame_id = obs_msg.header.frame_id;
    bscan_cloud_.width = obs_msg.ascan_length * obs_msg.num_ascans;
    bscan_cloud_.row_step = bscan_cloud_.point_step * bscan_cloud_.width;
    bscan_cloud_.data.clear();
    bscan_cloud_.data.resize(bscan_cloud_.row_step);

    sensor_msgs::PointCloud2Iterator<float> bscan_iterX(bscan_cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> bscan_iterY(bscan_cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> bscan_iterZ(bscan_cloud_, "z");
    sensor_msgs::PointCloud2Iterator<float> bscan_iterAmps(bscan_cloud_, "Amplitudes");

    gated_bscan_cloud_.header.stamp = obs_msg.header.stamp;
    gated_bscan_cloud_.header.frame_id = obs_msg.header.frame_id;
    gated_bscan_cloud_.width = obs_msg.ascan_length * obs_msg.num_ascans;
    gated_bscan_cloud_.row_step = gated_bscan_cloud_.point_step * gated_bscan_cloud_.width;
    gated_bscan_cloud_.data.clear();
    gated_bscan_cloud_.data.resize(gated_bscan_cloud_.row_step);

    sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterX(gated_bscan_cloud_, "x");
    sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterY(gated_bscan_cloud_, "y");
    sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterZ(gated_bscan_cloud_, "z");
    sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterAmps(gated_bscan_cloud_, "Amplitudes");
    sensor_msgs::PointCloud2Iterator<float> gated_bscan_iterTof(gated_bscan_cloud_, "TimeofFlight");

    float nan_value = std::numeric_limits<float>::quiet_NaN();
    float max_amp_f = (float)obs_msg.max_amplitude;
    float tcg_limit_pos = max_amp_f * tcg_limit_;
    float tcg_limit_neg = -max_amp_f * tcg_limit_;

    float x;
    float y;
    float z;
    float normalised_amplitude;
    float gated_amplitude;
    float tof;

    int element_i = 0;

    for (const auto& ascan : obs_msg.ascans) {
        bool    found_front_wall = false;
        float   depth_front_wall = nan_value;
        bool    found_back_wall  = false;

        int i = 0;
        for (auto amplitude : ascan.amplitudes) {
            x = 0.0f;
            y = lookups_valid_ ? y_lookup_[element_i] : (float)element_i * (float)obs_msg.element_pitch * 0.001f;
            z = lookups_valid_ ? z_lookup_[i] : (float)i * (float)obs_msg.vel_material * (1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f)) / 2.0f;

            if (lookups_valid_ && tcg_gain_[i] != 1.0f) {
                float amplitude_tcg = (float)amplitude * tcg_gain_[i];

                if (amplitude_tcg > tcg_limit_pos) {
                    amplitude_tcg = tcg_limit_pos;
                } else if (amplitude_tcg < tcg_limit_neg) {
                    amplitude_tcg = tcg_limit_neg;
                }

                amplitude = amplitude_tcg;
            }

            normalised_amplitude = (float)amplitude / max_amp_f;

            *bscan_iterX = x;
            *bscan_iterY = y;
            *bscan_iterZ = z;
            *bscan_iterAmps = normalised_amplitude;

            ++bscan_iterX;
            ++bscan_iterY;
            ++bscan_iterZ;
            ++bscan_iterAmps;

            // Front wall gate
            if (!found_front_wall and
                normalised_amplitude > gate_front_wall_) {
                depth_front_wall = z;
                if (zero_to_front_wall_) {
                    z = 0.0f;
                }
                gated_amplitude = normalised_amplitude;

                found_front_wall = true;
                if (!show_front_wall_) {
                    x               = nan_value;
                    y               = nan_value;
                    z               = nan_value;
                    gated_amplitude = nan_value;
                    tof             = nan_value;
                }

            // Back wall gate
            } else if (found_front_wall and
                       !found_back_wall and
                       z < max_depth_ and
                       z > (depth_to_skip_ + depth_front_wall) and
                       normalised_amplitude > gate_back_wall_) {
                if (zero_to_front_wall_) {
                    z = z - depth_front_wall;
                }
                gated_amplitude = normalised_amplitude;
                tof = z;

                found_back_wall = true;
            } else {
                x               = nan_value;
                y               = nan_value;
                z               = nan_value;
                gated_amplitude = nan_value;
                tof             = nan_value;
            }

            *gated_bscan_iterX    = x;
            *gated_bscan_iterY    = y;
            *gated_bscan_iterZ    = z;
            *gated_bscan_iterAmps = gated_amplitude;
            *gated_bscan_iterTof  = tof;

            ++gated_bscan_iterX;
            ++gated_bscan_iterY;
            ++gated_bscan_iterZ;
            ++gated_bscan_iterAmps;
            ++gated_bscan_iterTof;

            ++i;
        }
        ++element_i;
    }
}


void PeakComponent::timerCb() {
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 600000, node_name_ << ": Node running");
    if (stream_) {
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 60000, node_name_ << ": Streaming data...");
        std::lock_guard<std::mutex> lock(processing_mutex_);
        // Consume the latest frame produced by the asynchronous acquisition.
        if (peak_handler_.getLatestData(latest_data_)) {
            processMeasurement();
        }
    } else {
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 60000, node_name_ << ": Not streaming data...");
    }
}


} // namespace peak_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(peak_namespace::PeakComponent)
