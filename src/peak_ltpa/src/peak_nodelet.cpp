#include "peak_nodelet.h"


namespace peak_namespace {

PeakNodelet::PeakNodelet()
  : rate_(10),
    ns_("/peak"), // TODO: Figure out how to get this when using nodelets so it isn't hardcoded
    peak_handler_(),
    stream_(false)
{
}


void PeakNodelet::onInit()
{
    NODELET_INFO_STREAM(node_name_ << ": Initialising node...");

    ros::NodeHandle &nh_ = getMTNodeHandle();
    node_name_ = getName();
    //ns_ = ros::this_node::getNamespace();
    package_path_ = ros::package::getPath("peak_ltpa");
    peak_handler_.setup(
                        PeakNodelet::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_),
                        PeakNodelet::paramHandler(ns_ + "/settings/peak_address", peak_address_),
                        PeakNodelet::paramHandler(ns_ + "/settings/peak_port", peak_port_),
                        package_path_ + "/mps/" + PeakNodelet::paramHandler(ns_ + "/settings/mps_file", mps_file_)
                        );

   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   // TODO: Move to using smart pointers
   // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ltpa_data_ptr_ = peak_handler_.ltpa_data_ptr();

    rate_ = ros::Rate(acquisition_rate_);
    PeakNodelet::paramHandler(ns_ + "/settings/digitisation_rate", digitisation_rate_);

    initHardware();

    ascan_publisher_ = nh_.advertise<peak_ltpa::Observation>(ns_ + "/a_scans", 10000, true);
    bscan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/b_scan", 10000, true);

    single_measure_service_ = nh_.advertiseService(ns_ + "/take_single_measurement", &PeakNodelet::takeMeasurementSrvCb, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &PeakNodelet::streamDataSrvCb, this);

    timer_ = nh_.createTimer(ros::Duration(1 / acquisition_rate_), &PeakNodelet::timerCb, this);

    NODELET_INFO_STREAM(node_name_ << ": Node initialised");
}


//PeakNodelet::~PeakNodelet() {
//}


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
    peak_handler_.sendReset(digitisation_rate_);
    peak_handler_.readMpsFile();
    peak_handler_.sendMpsConfiguration();

    prePopulateMessage();

    NODELET_INFO_STREAM(node_name_ << ": Peak hardware initialised");
}


void PeakNodelet::prePopulateMessage() {
    PeakNodelet::paramHandler(ns_ + "/settings/frame_id", ltpa_msg_.header.frame_id);

    // Get settings the PeakHandler extracted from the .mps file
    ltpa_msg_.dof = peak_handler_.dof_;
    ltpa_msg_.gate_start = peak_handler_.gate_start_;
    ltpa_msg_.gate_end = peak_handler_.gate_end_;
    ltpa_msg_.ascan_length = peak_handler_.ascan_length_;
    ltpa_msg_.num_ascans = peak_handler_.num_a_scans_;
    ltpa_msg_.ascans.reserve(ltpa_msg_.num_ascans);

    ltpa_msg_.digitisation_rate = ltpa_data_ptr_->digitisation_rate;

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


bool PeakNodelet::streamDataSrvCb(peak_ltpa::StreamData::Request& request,
                                  peak_ltpa::StreamData::Response& response) {
    NODELET_INFO_STREAM(node_name_ << ": Streaming request received: " << request.stream_data);
    if (request.stream_data) {
        stream_ = true;
        response.success = true;
        return true;
    } else {
        stream_ = false;
        response.success = true;
        return true;
    }
}


bool PeakNodelet::takeMeasurementSrvCb(peak_ltpa::TakeSingleMeasurement::Request& request,
                                       peak_ltpa::TakeSingleMeasurement::Response& response) {
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
    // auto begin = std::chrono::high_resolution_clock::now();

    // TODO: Return some validity flag for what is in the ltpa_data_ptr_
    if (peak_handler_.sendDataRequest()) {

        // auto end_1 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [peak_handler_.sendDataRequest()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_1-begin).count() << " us" << std::endl;
        // std::cout << "\033[0m";

        populateAScanMessage();

        // auto end_2 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [PeakNodelet::populateAScanMessage()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_2-end_1).count() << " us" << std::endl;
        // std::cout << "\033[0m";

        ascan_publisher_.publish(ltpa_msg_);

        // auto end_3 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [ascan_publisher_.publish(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_3-end_2).count() << " us" << std::endl;
        // std::cout << "\033[0m";

        populateBScanMessage(ltpa_msg_);

        // auto end_4 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [PeakNodelet::populateBScanMessage(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_4-end_3).count() << " us" << std::endl;
        // std::cout << "\033[0m";
    }

    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "\033[32m";
    // std::cout << "Profiling [PeakNodelet::takeMeasurement()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
    // std::cout << "\033[0m";
}


void PeakNodelet::populateAScanMessage() {
    ltpa_msg_.header.stamp = ros::Time::now();
    ltpa_msg_.ascans.clear();

    // TODO: Consider matching the peak_ltpa::Ascan and PeakHandler::DofMessage
    //       and set appropriate ROS message attributes
    for (auto ascan : ltpa_data_ptr_->ascans) {
        peak_ltpa::Ascan ascan_msg;
        ascan_msg.count = ascan.header.count;
        ascan_msg.test_number = ascan.header.testNo;
        ascan_msg.dof = ascan.header.dof;
        ascan_msg.channel = ascan.header.channel;
        // ascan_msg.amplitudes = ascan.amps;
        if (ascan.header.dof == 1) {
            ascan_msg.amplitudes_8 = ascan.amps_8;
        } else if (ascan.header.dof == 4) {
            ascan_msg.amplitudes_16 = ascan.amps_16;
        }
        ltpa_msg_.ascans.push_back(ascan_msg);
    }

    // ltpa_msg_.max_amplitude = ltpa_data_ptr_->max_amplitude;
    if (ltpa_msg_.dof == 1) {
        ltpa_msg_.max_amplitude_8 = ltpa_data_ptr_->max_amplitude_8;
    } else if (ltpa_msg_.dof == 4) {
        ltpa_msg_.max_amplitude_16 = ltpa_data_ptr_->max_amplitude_16;
    }
}


void PeakNodelet::populateBScanMessage(const peak_ltpa::Observation& obs_msg) {
    sensor_msgs::PointCloud2 bscan_cloud;
    bscan_cloud.header.stamp = obs_msg.header.stamp;
    bscan_cloud.header.frame_id = obs_msg.header.frame_id;

    // TODO: Move these instantiations out of here
    sensor_msgs::PointCloud2Modifier bscan_cloud_modifier(bscan_cloud);
    bscan_cloud_modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "z", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "Amplitudes", 1, sensor_msgs::PointField::FLOAT32       // 32 bits = 4 bytes
        );

    bscan_cloud.height = 1;
    bscan_cloud.width = obs_msg.ascan_length * obs_msg.num_ascans;
    bscan_cloud.is_dense = true;
    bscan_cloud.point_step = 16; // 4 fields * 4 bytes per field
    bscan_cloud.row_step = bscan_cloud.point_step * bscan_cloud.width;
    bscan_cloud.data.resize(bscan_cloud.row_step);

    // Iterators
    sensor_msgs::PointCloud2Iterator<float>      iterX(bscan_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>      iterY(bscan_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>      iterZ(bscan_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float>      iterAmps(bscan_cloud, "Amplitudes");

    //==================================================================================
    // TODO: Check all your units
    //==================================================================================

    double dt = (double)1.0 / (double)(obs_msg.digitisation_rate * (double)1000000.0);         // sec
    double time_in_wedge =       2.0 * obs_msg.wedge_depth / obs_msg.vel_wedge / 1000.0;       // sec
    double time_in_couplant =    2.0 * obs_msg.couplant_depth / obs_msg.vel_couplant / 1000.0; // sec
    double time_in_specimen =    2.0 * obs_msg.specimen_depth / obs_msg.vel_material / 1000.0; // sec

    // std::cout << "=====================================" << std::endl;
    // std::cout << (double)obs_msg.digitisation_rate << std::endl; // 100
    // std::cout << dt << std::endl;
    // std::cout << time_in_wedge << std::endl;
    // std::cout << time_in_couplant << std::endl;
    // std::cout << time_in_specimen << std::endl;
    // std::cout << "=====================================" << std::endl;

    // https://robotics.stackexchange.com/questions/87043/unable-to-remove-nan-points-using-function-removenanfrompointcloud
    // https://robotics.stackexchange.com/questions/90595/removing-single-points-from-a-sensor-msgspointcloud2
    float nan_value = std::numeric_limits<float>::quiet_NaN();

    float x;
    float y;
    float z;
    float normalised_amplitude;

    int element_i(0);
    for (const auto& ascan : obs_msg.ascans) {
        int i = 0;

        // std::vector amplitudes;
        // TODO: Don't do this copy operation
        // if (obs_msg.dof == 1) {
            //std::vector<int8_t> amplitudes;
            // amplitudes = ascan.amplitudes_8;
        // } else if (obs_msg.dof == 4) {
            //std::vector<int16_t> amplitudes;
            // amplitudes = ascan.amplitudes_16;
        // }

        // for (auto amplitude : ascan.amplitudes_8) {
        for (auto amplitude : ascan.amplitudes_16) {
            // Raw Amplitude
            // normalised_amplitude = (float)amplitude;

            // Normalised on Linear Scale
            if (obs_msg.dof == 1) {
                normalised_amplitude = (float)amplitude / (float)obs_msg.max_amplitude_8;
            } else if (obs_msg.dof == 4) {
                normalised_amplitude = (float)amplitude / (float)obs_msg.max_amplitude_16;
            }
            // normalised_amplitude = (float)abs( (float)amplitude / (float)obs_msg.max_amplitude);

            // Normalised on dB Scale (log)
            // normalised_amplitude = 20.0 * (float)log10( (float)abs( (float)amplitude / (float)obs_msg.max_amplitude) );

            // if (abs(normalised_amplitude) < 0.6) {
            // if (abs(normalised_amplitude) > 0.5) {
            if (true) {
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
                x = (float)0.0;
                y = (float)(element_i * obs_msg.element_pitch * 0.001); // mm to m
                z = (float)(i * obs_msg.vel_material * dt / (double)2.0);
            } else {
                x = nan_value;
                y = nan_value;
                z = nan_value;
                normalised_amplitude = nan_value;
            }

            *iterX = x;
            *iterY = y;
            *iterZ = z;
            *iterAmps = normalised_amplitude;

            ++iterX;
            ++iterY;
            ++iterZ;
            ++iterAmps;
            ++i;
        }
        ++element_i;
    }
    bscan_publisher_.publish(bscan_cloud);
}


void PeakNodelet::timerCb(const ros::TimerEvent& event){
    NODELET_INFO_STREAM_THROTTLE(600, node_name_ << ": Node running");
    if (stream_) {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Streaming data...");
        takeMeasurement();
    } else {
        NODELET_INFO_STREAM_THROTTLE(60, node_name_ << ": Not streaming data...");
    }
}


} // namespace peak_namespace

PLUGINLIB_EXPORT_CLASS(peak_namespace::PeakNodelet, nodelet::Nodelet);