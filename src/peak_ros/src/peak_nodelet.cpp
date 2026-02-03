#include "peak_nodelet.h"


namespace peak_namespace {

PeakNodelet::PeakNodelet()
  : rate_(10),
    ns_("/peak"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    peak_handler_()
{
}


PeakNodelet::~PeakNodelet() {
    peak_handler_.stopAsyncAcquisition();
}


void PeakNodelet::onInit()
{
    NODELET_INFO_STREAM(node_name_ << ": Initialising node...");

    ros::NodeHandle &nh_ = getMTNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();
    package_path_ = ros::package::getPath("peak_ros");
    peak_handler_.setup(
                        PeakNodelet::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_),
                        PeakNodelet::paramHandler(ns_ + "/settings/peak_address", peak_address_),
                        PeakNodelet::paramHandler(ns_ + "/settings/peak_port", peak_port_),
                        package_path_ + "/mps/" + PeakNodelet::paramHandler(ns_ + "/settings/mps_file", mps_file_)
                        );

    rate_ = ros::Rate(acquisition_rate_);
    PeakNodelet::paramHandler(ns_ + "/settings/digitisation_rate", digitisation_rate_);
    PeakNodelet::paramHandler(ns_ + "/settings/profile", profile_);

    PeakNodelet::paramHandler(ns_ + "/settings/tcg/use_tcg", use_tcg_);
    PeakNodelet::paramHandler(ns_ + "/settings/tcg/amp_factor", amp_factor_);
    PeakNodelet::paramHandler(ns_ + "/settings/tcg/depth_factor", depth_factor_);
    PeakNodelet::paramHandler(ns_ + "/settings/tcg/tcg_limit", tcg_limit_);
    depth_factor_ = depth_factor_ * 0.001f;

    PeakNodelet::paramHandler(ns_ + "/settings/gates/gate_front_wall", gate_front_wall_);
    PeakNodelet::paramHandler(ns_ + "/settings/gates/depth_to_skip", depth_to_skip_);
    PeakNodelet::paramHandler(ns_ + "/settings/gates/gate_back_wall", gate_back_wall_);
    PeakNodelet::paramHandler(ns_ + "/settings/gates/max_depth", max_depth_);
    PeakNodelet::paramHandler(ns_ + "/settings/gates/zero_to_front_wall", zero_to_front_wall_);
    PeakNodelet::paramHandler(ns_ + "/settings/gates/show_front_wall", show_front_wall_);
    depth_to_skip_ = depth_to_skip_ * 0.001f;
    max_depth_ = max_depth_ * 0.001f;

    initHardware();

    prePopulateAScanMessage();
    precomputeBScanLookups();
    prePopulateBScanMessage();
    prePopulateGatedBScanMessage();

    ascan_publisher_ =        nh_.advertise<peak_ros::Observation>(ns_ + "/a_scans", 3, false);
    bscan_publisher_ =        nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/b_scan", 3, false);
    gated_bscan_publisher_ =  nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/gated_b_scan", 3, false);

    single_measure_service_ = nh_.advertiseService(ns_ + "/take_single_measurement", &PeakNodelet::takeMeasurementSrvCb, this);
    stream_service_ =         nh_.advertiseService(ns_ + "/stream_data", &PeakNodelet::streamDataSrvCb, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / (double)acquisition_rate_), &PeakNodelet::timerCb, this);

    NODELET_INFO_STREAM(node_name_ << ": Node initialised");
}


template <typename ParamType>
ParamType PeakNodelet::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name)) {
        NODELET_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    NODELET_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


void PeakNodelet::initHardware() {
    NODELET_INFO_STREAM(node_name_ << ": Initialising Peak hardware...");

    peak_handler_.connect();

    int reset_sleep = 10;
    nh_.param(ns_ + "/settings/reset_sleep_seconds", reset_sleep, 10);
    peak_handler_.sendReset(digitisation_rate_, reset_sleep);

    peak_handler_.readMpsFile();
    peak_handler_.sendMpsConfiguration();

    NODELET_INFO_STREAM(node_name_ << ": Peak hardware initialised");
}


void PeakNodelet::prePopulateAScanMessage() {
    PeakNodelet::paramHandler(ns_ + "/settings/frame_id", ltpa_msg_.header.frame_id);

    // Get settings the PeakHandler extracted from the .mps file
    ltpa_msg_.dof = peak_handler_.dof_;
    ltpa_msg_.gate_start = peak_handler_.gate_start_;
    ltpa_msg_.gate_end = peak_handler_.gate_end_;
    ltpa_msg_.ascan_length = peak_handler_.ascan_length_;
    ltpa_msg_.num_ascans = peak_handler_.num_a_scans_;
    ltpa_msg_.ascans.reserve(ltpa_msg_.num_ascans);

    // digitisation_rate is set during sendReset, read it from the handler's data pointer
    ltpa_msg_.digitisation_rate = peak_handler_.ltpa_data_ptr()->digitisation_rate;

    // TODO: Consider sending this as a separate one time latched message rather than repeated here
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/n_elements", ltpa_msg_.n_elements);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/element_pitch", ltpa_msg_.element_pitch);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/inter_element_spacing", ltpa_msg_.inter_element_spacing);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/element_width", ltpa_msg_.element_width);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/vel_wedge", ltpa_msg_.vel_wedge);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/vel_couplant", ltpa_msg_.vel_couplant);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/vel_material", ltpa_msg_.vel_material);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/wedge_angle", ltpa_msg_.wedge_angle);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/wedge_depth", ltpa_msg_.wedge_depth);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/couplant_depth", ltpa_msg_.couplant_depth);
    PeakNodelet::paramHandler(ns_ + "/settings/boundary_conditions/specimen_depth", ltpa_msg_.specimen_depth);

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


void PeakNodelet::precomputeBScanLookups() {
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


void PeakNodelet::prePopulateBScanMessage() {
    int fields          = 4;
    int bytes_per_field = 4; // 32 bits = 4 bytes

    sensor_msgs::PointCloud2Modifier bscan_cloud_modifier(bscan_cloud_);
    bscan_cloud_modifier.setPointCloud2Fields(
        fields,
        "x", 1,          sensor_msgs::PointField::FLOAT32,
        "y", 1,          sensor_msgs::PointField::FLOAT32,
        "z", 1,          sensor_msgs::PointField::FLOAT32,
        "Amplitudes", 1, sensor_msgs::PointField::FLOAT32
        );
    bscan_cloud_.height = 1;
    bscan_cloud_.is_dense = true;
    bscan_cloud_.point_step = fields * bytes_per_field;
}


void PeakNodelet::prePopulateGatedBScanMessage() {
    int fields          = 5;
    int bytes_per_field = 4; // 32 bits = 4 bytes

    sensor_msgs::PointCloud2Modifier gated_bscan_cloud_modifier(gated_bscan_cloud_);
    gated_bscan_cloud_modifier.setPointCloud2Fields(
        fields,
        "x", 1,            sensor_msgs::PointField::FLOAT32,
        "y", 1,            sensor_msgs::PointField::FLOAT32,
        "z", 1,            sensor_msgs::PointField::FLOAT32,
        "Amplitudes", 1,   sensor_msgs::PointField::FLOAT32,
        "TimeofFlight", 1, sensor_msgs::PointField::FLOAT32
        );
    gated_bscan_cloud_.height = 1;
    gated_bscan_cloud_.is_dense = true;
    gated_bscan_cloud_.point_step = fields * bytes_per_field;
}


bool PeakNodelet::streamDataSrvCb(peak_ros::StreamData::Request& request,
                                  peak_ros::StreamData::Response& response) {
    NODELET_INFO_STREAM(node_name_ << ": Streaming request received: " << request.stream_data);
    if (request.stream_data) {
        stream_ = true;
        peak_handler_.startAsyncAcquisition(
            [this](bool valid) { onDataReady(valid); },
            acquisition_rate_);
        response.success = true;
        return true;
    } else {
        stream_ = false;
        peak_handler_.stopAsyncAcquisition();
        response.success = true;
        return true;
    }
}


bool PeakNodelet::takeMeasurementSrvCb(peak_ros::TakeSingleMeasurement::Request& request,
                                       peak_ros::TakeSingleMeasurement::Response& response) {
    NODELET_INFO_STREAM(node_name_ << ": Take single measurement request received: " << request.take_single_measurement);
    if (request.take_single_measurement) {
        takeMeasurement();
        response.success = true;
        return true;
    } else {
        response.success = false;
        return true;
    }
}


void PeakNodelet::takeMeasurement() {
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
        std::cout << "Profiling [PeakNodelet::takeMeasurement()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }
}


void PeakNodelet::processMeasurement() {
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
        std::cout << "Profiling [PeakNodelet::populateAScanMessage()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_1-begin).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    ascan_publisher_.publish(ltpa_msg_);

    if (profile_) {
        end_2 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [ascan_publisher_.publish(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_2-end_1).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    populateBScanMessage(ltpa_msg_);

    if (profile_) {
        end_3 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [PeakNodelet::populateBScanMessage(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_3-end_2).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }

    bscan_publisher_.publish(bscan_cloud_);
    gated_bscan_publisher_.publish(gated_bscan_cloud_);

    if (profile_) {
        end_4 = std::chrono::high_resolution_clock::now();
        std::cout << "\033[32m";
        std::cout << "Profiling [publish bscan + gated] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_4-end_3).count() << " us" << std::endl;
        std::cout << "\033[0m";
    }
}


void PeakNodelet::populateAScanMessage() {
    ltpa_msg_.header.stamp = ros::Time::now();
    ltpa_msg_.ascans.clear();

    for (auto& ascan : latest_data_.ascans) {
        peak_ros::Ascan ascan_msg;
        ascan_msg.count = ascan.header.count;
        ascan_msg.test_number = ascan.header.testNo;
        ascan_msg.dof = ascan.header.dof;
        ascan_msg.channel = ascan.header.channel;
        ascan_msg.amplitudes = std::move(ascan.amps);
        ltpa_msg_.ascans.push_back(std::move(ascan_msg));
    }

    ltpa_msg_.max_amplitude = latest_data_.max_amplitude;
}


void PeakNodelet::populateBScanMessage(const peak_ros::Observation& obs_msg) {
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
        float   amp_front_wall   = nan_value;
        float   depth_front_wall = nan_value;
        bool    found_back_wall  = false;
        float   amp_back_wall    = nan_value;
        float   depth_back_wall  = nan_value;

        y = lookups_valid_ ? y_lookup_[element_i] : (float)element_i * (float)obs_msg.element_pitch * 0.001f;

        int i = 0;
        for (auto amplitude : ascan.amplitudes) {
            x = 0.0f;
            z = lookups_valid_ ? z_lookup_[i] : 0.0f;

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


void PeakNodelet::onDataReady(bool /*valid*/) {
    // Called from the async I/O thread when new data arrives.
    // Schedule a one-shot timer with zero duration to process data
    // on the ROS callback queue immediately, rather than waiting
    // for the next periodic timer tick (up to 50ms latency saved).
    ros::NodeHandle& nh = getMTNodeHandle();
    nh.createTimer(ros::Duration(0.0),
        [this](const ros::TimerEvent&) {
            std::lock_guard<std::mutex> lock(processing_mutex_);
            if (stream_ && peak_handler_.getLatestData(latest_data_)) {
                processMeasurement();
            }
        }, true /* oneshot */, true /* autostart */);
}


void PeakNodelet::timerCb(const ros::TimerEvent& /*event*/){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
    if (stream_) {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Streaming data...");
        std::lock_guard<std::mutex> lock(processing_mutex_);
        if (peak_handler_.getLatestData(latest_data_)) {
            processMeasurement();
        }
    } else {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Not streaming data...");
    }
}


} // namespace peak_namespace

PLUGINLIB_EXPORT_CLASS(peak_namespace::PeakNodelet, nodelet::Nodelet);
