#include "peak_component.hpp"

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>


namespace peak_namespace {

PeakComponent::PeakComponent(const rclcpp::NodeOptions& options)
  : rclcpp::Node("peak_node", options),
    peak_handler_(),
    stream_(false)
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

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO: Move to using smart pointers, mutex, futures or semiphors
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ltpa_data_ptr_ = peak_handler_.ltpa_data_ptr();

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
    prePopulateBScanMessage();
    prePopulateGatedBScanMessage();

    rclcpp::QoS latched_qos = rclcpp::QoS(100).transient_local();

    ascan_publisher_       = create_publisher<peak_ros::msg::Observation>("a_scans", latched_qos);
    bscan_publisher_       = create_publisher<sensor_msgs::msg::PointCloud2>("b_scan", latched_qos);
    gated_bscan_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("gated_b_scan", latched_qos);

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


void PeakComponent::initHardware() {
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Initialising Peak hardware...");

    peak_handler_.connect();
    peak_handler_.sendReset(digitisation_rate_);
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

    ltpa_msg_.digitisation_rate = ltpa_data_ptr_->digitisation_rate;

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
        response->success = true;
    } else {
        stream_ = false;
        response->success = true;
    }
}


void PeakComponent::takeMeasurementSrvCb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;
    RCLCPP_INFO_STREAM(get_logger(), node_name_ << ": Take single measurement request received");
    takeMeasurement();
    response->success = true;
    response->message = "Single measurement taken";
}


void PeakComponent::takeMeasurement() {
    // TODO: Remove profiling when happy with acquisition rates
    std::chrono::high_resolution_clock::time_point begin;
    std::chrono::high_resolution_clock::time_point end;
    std::chrono::high_resolution_clock::time_point end_1;
    std::chrono::high_resolution_clock::time_point end_2;
    std::chrono::high_resolution_clock::time_point end_3;
    std::chrono::high_resolution_clock::time_point end_4;
    std::chrono::high_resolution_clock::time_point end_5;

    if (profile_) begin = std::chrono::high_resolution_clock::now();

    // ~40ms
    if (peak_handler_.sendDataRequest()) {

        if (profile_) {
            end_1 = std::chrono::high_resolution_clock::now();
            std::cout << "\033[32m";
            std::cout << "Profiling [peak_handler_.sendDataRequest()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_1-begin).count() << " us" << std::endl;
            std::cout << "\033[0m";
        }

        // ~0.3ms
        populateAScanMessage();

        if (profile_) {
            end_2 = std::chrono::high_resolution_clock::now();
            std::cout << "\033[32m";
            std::cout << "Profiling [PeakComponent::populateAScanMessage()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_2-end_1).count() << " us" << std::endl;
            std::cout << "\033[0m";
        }

        // ~0.4ms
        ascan_publisher_->publish(ltpa_msg_);

        if (profile_) {
            end_3 = std::chrono::high_resolution_clock::now();
            std::cout << "\033[32m";
            std::cout << "Profiling [ascan_publisher_->publish(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_3-end_2).count() << " us" << std::endl;
            std::cout << "\033[0m";
        }

        // ~12ms
        populateBScanMessage(ltpa_msg_);

        if (profile_) {
            end_4 = std::chrono::high_resolution_clock::now();
            std::cout << "\033[32m";
            std::cout << "Profiling [PeakComponent::populateBScanMessage(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_4-end_3).count() << " us" << std::endl;
            std::cout << "\033[0m";
        }

        bscan_publisher_->publish(bscan_cloud_);

        if (profile_) {
            end_5 = std::chrono::high_resolution_clock::now();
            std::cout << "\033[32m";
            std::cout << "Profiling [bscan_publisher_->publish(bscan_cloud_);] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_5-end_4).count() << " us" << std::endl;
            std::cout << "\033[0m";
        }

        gated_bscan_publisher_->publish(gated_bscan_cloud_);
    }

    if (profile_) {
        end = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [PeakComponent::takeMeasurement()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }
}


void PeakComponent::populateAScanMessage() {
    ltpa_msg_.header.stamp = this->now();
    ltpa_msg_.ascans.clear();

    for (auto ascan : ltpa_data_ptr_->ascans) {
        peak_ros::msg::Ascan ascan_msg;
        ascan_msg.count = ascan.header.count;
        ascan_msg.test_number = ascan.header.testNo;
        ascan_msg.dof = ascan.header.dof;
        ascan_msg.channel = ascan.header.channel;
        ascan_msg.amplitudes = ascan.amps;
        ltpa_msg_.ascans.push_back(ascan_msg);
    }

    ltpa_msg_.max_amplitude = ltpa_data_ptr_->max_amplitude;
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

    float dt =                  1.0f / ((float)obs_msg.digitisation_rate * 1000000.0f);        // sec
    // double time_in_wedge =       2.0 * obs_msg.wedge_depth / obs_msg.vel_wedge / 1000.0;       // sec
    // double time_in_couplant =    2.0 * obs_msg.couplant_depth / obs_msg.vel_couplant / 1000.0; // sec
    // double time_in_specimen =    2.0 * obs_msg.specimen_depth / obs_msg.vel_material / 1000.0; // sec

    float nan_value = std::numeric_limits<float>::quiet_NaN();

    float x;
    float y;
    float z;
    float normalised_amplitude;
    float gated_amplitude;
    float tof;

    int element_i = 0;

    for (const auto& ascan : obs_msg.ascans) {
        bool    found_front_wall = false;
        float   amp_front_wall   = nan_value;
        float   depth_front_wall = nan_value;
        bool    found_back_wall  = false;
        float   amp_back_wall    = nan_value;
        float   depth_back_wall  = nan_value;

        int i = 0;
        for (auto amplitude : ascan.amplitudes) {
            // GAT(S) --- GAT <Tn> <Gate Start> <Gate End>
            // Defines search gate start and end positions for the specified test.
            // By default, the gate units are in machine units.
            // A machine unit is defined by the digitisation rate (i.e. 10nSec for 100MHz digitisation).
            // Maybe assume 100 MHz to start with...
            // double t = (double)i * dt;
            // double z = 0.0;
            // if (t < time_in_wedge) {
            //     z = t * obs_msg.vel_wedge;
            // } else if (t < time_in_couplant) {
            //     z = (t - time_in_wedge) * obs_msg.vel_couplant
            //          + obs_msg.wedge_depth;
            // } else if (t < time_in_specimen) {
            //     z = (t - time_in_wedge - time_in_couplant) * obs_msg.vel_couplant
            //          + obs_msg.wedge_depth
            //          + obs_msg.couplant_depth;
            // }

            x = 0.0f;
            y = (float)((float)element_i * (float)obs_msg.element_pitch * 0.001f); // mm to m
            z = (float)((float)i * (float)obs_msg.vel_material * dt / 2.0f);

            if (use_tcg_ and z > (10.0f * 0.001f)) { // TODO: Param for skipping x mm in before applying tcg
                float amplitude_tcg;

                // Amplify by n dB per l mm
                // amplitude_tcg = amplitude * 10.0 ^ (n * (z / l) / 20.0);
                amplitude_tcg = (float)amplitude * std::pow(10.0f, (amp_factor_ * (z / depth_factor_) / 20.0f));


                if (amplitude_tcg > (float)obs_msg.max_amplitude * tcg_limit_) {
                    amplitude_tcg = (float)obs_msg.max_amplitude * tcg_limit_;
                } else if (amplitude_tcg < -(float)obs_msg.max_amplitude * tcg_limit_) {
                    amplitude_tcg = -(float)obs_msg.max_amplitude * tcg_limit_;
                }

                amplitude = amplitude_tcg;
            }

            // Raw Amplitude
            // normalised_amplitude = (float)amplitude;

            // Normalised on Linear Scale
            normalised_amplitude = (float)amplitude / (float)obs_msg.max_amplitude;

            // Normalised on dB Scale
            // normalised_amplitude = 20.0 * (float)log10( (float)abs( (float)amplitude / (float)obs_msg.max_amplitude) );

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
                amp_front_wall = normalised_amplitude;
                gated_amplitude = amp_front_wall;

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
                depth_back_wall = z;
                if (zero_to_front_wall_) {
                    z = z - depth_front_wall;
                }
                amp_back_wall = normalised_amplitude;
                gated_amplitude = amp_back_wall;
                tof = depth_back_wall;

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
        takeMeasurement();
    } else {
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 60000, node_name_ << ": Not streaming data...");
    }
}


} // namespace peak_namespace

RCLCPP_COMPONENTS_REGISTER_NODE(peak_namespace::PeakComponent)
