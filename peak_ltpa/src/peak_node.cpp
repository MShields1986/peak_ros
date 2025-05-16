#include "peak_node.h"


PeakNode::PeakNode(bool* kill_now)
    :  nh_(),
       rate_(10),
       kill_now_(kill_now),
       node_name_(ros::this_node::getName()),
       ns_(ros::this_node::getNamespace()),
       package_path_(ros::package::getPath("peak_ltpa")),
       peak_handler_(
            PeakNode::paramHandler(ns_ + "/settings/acquisition_rate", acquisition_rate_),
            PeakNode::paramHandler(ns_ + "/settings/peak_address", peak_address_),
            PeakNode::paramHandler(ns_ + "/settings/peak_port", peak_port_),
            package_path_ + "/mps/" + PeakNode::paramHandler(ns_ + "/settings/mps_file", mps_file_)
            ),
       // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       // TODO: Move to using smart pointers
       // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
       ltpa_data_ptr_(peak_handler_.ltpa_data_ptr()),
       stream_(false)

{
    ROS_INFO_STREAM(node_name_ << ": Initialising node...");
    rate_ = ros::Rate(acquisition_rate_);

    initHardware();

    ascan_publisher_ = nh_.advertise<peak_ltpa::Observation>(ns_ + "/a_scans", 10000, true);
    bscan_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(ns_ + "/b_scan", 10000, true);

    single_measure_service_ = nh_.advertiseService(ns_ + "/take_single_measurement", &PeakNode::takeMeasurementSrvCb, this);
    stream_service_ = nh_.advertiseService(ns_ + "/stream_data", &PeakNode::streamDataSrvCb, this);

    ROS_INFO_STREAM(node_name_ << ": Node initialised");
}


//PeakNode::~PeakNode() {
//}


template <typename ParamType>
ParamType PeakNode::paramHandler(std::string param_name, ParamType& param_value) {
    while (!nh_.hasParam(param_name) and !*kill_now_) {
        ROS_INFO_STREAM_THROTTLE(10, node_name_ << ": Waiting for parameter " << param_name);
    }
    nh_.getParam(param_name, param_value);
    ROS_DEBUG_STREAM(node_name_ << ": Read in parameter " << param_name << " = " << param_value);
    return param_value;
}


void PeakNode::initHardware() {
    ROS_INFO_STREAM(node_name_ << ": Initialising Peak hardware...");

    peak_handler_.readMpsFile();
    peak_handler_.connect(100);
    peak_handler_.sendMpsConfiguration();

    prePopulateMessage();

    ROS_INFO_STREAM(node_name_ << ": Peak hardware initialised");
}


void PeakNode::prePopulateMessage() {
    PeakNode::paramHandler(ns_ + "/settings/frame_id", ltpa_msg_.header.frame_id);

    // Get settings the PeakHandler extracted from the .mps file
    ltpa_msg_.dof = peak_handler_.dof_;
    ltpa_msg_.gate_start = peak_handler_.gate_start_;
    ltpa_msg_.gate_end = peak_handler_.gate_end_;
    ltpa_msg_.ascan_length = peak_handler_.ascan_length_;
    ltpa_msg_.num_ascans = peak_handler_.num_a_scans_;
    ltpa_msg_.ascans.reserve(ltpa_msg_.num_ascans);

    ltpa_msg_.digitisation_rate = ltpa_data_ptr_->digitisation_rate;

    // TODO: Consider sending this as a separate one time latched message rather than repeated here
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/n_elements", ltpa_msg_.n_elements);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/element_pitch", ltpa_msg_.element_pitch);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/inter_element_spacing", ltpa_msg_.inter_element_spacing);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/element_width", ltpa_msg_.element_width);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/vel_wedge", ltpa_msg_.vel_wedge);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/vel_couplant", ltpa_msg_.vel_couplant);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/vel_material", ltpa_msg_.vel_material);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/wedge_angle", ltpa_msg_.wedge_angle);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/wedge_depth", ltpa_msg_.wedge_depth);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/couplant_depth", ltpa_msg_.couplant_depth);
    PeakNode::paramHandler(ns_ + "/settings/reconstruction/specimen_depth", ltpa_msg_.specimen_depth);

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


bool PeakNode::streamDataSrvCb(peak_ltpa::StreamData::Request& request,
                               peak_ltpa::StreamData::Response& response) {
    ROS_INFO_STREAM(node_name_ << ": Streaming request received: " << request.stream_data);
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


bool PeakNode::takeMeasurementSrvCb(peak_ltpa::TakeSingleMeasurement::Request& request,
                                    peak_ltpa::TakeSingleMeasurement::Response& response) {
    ROS_INFO_STREAM(node_name_ << ": Take single measurement request received: " << request.take_single_measurement);
    if (request.take_single_measurement) {
        takeMeasurement();
        response.success = true;
        return true;
    } else {
        response.success = false;
        return true;
    }
}


void PeakNode::takeMeasurement() {
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
        // std::cout << "Profiling [PeakNode::populateAScanMessage()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_2-end_1).count() << " us" << std::endl;
        // std::cout << "\033[0m";

        ascan_publisher_.publish(ltpa_msg_);

        // auto end_3 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [ascan_publisher_.publish(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_3-end_2).count() << " us" << std::endl;
        // std::cout << "\033[0m";

        populateBScanMessage(ltpa_msg_);

        // auto end_4 = std::chrono::high_resolution_clock::now();
        // std::cout << "\033[32m";
        // std::cout << "Profiling [PeakNode::populateBScanMessage(ltpa_msg_)] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end_4-end_3).count() << " us" << std::endl;
        // std::cout << "\033[0m";
    }

    // auto end = std::chrono::high_resolution_clock::now();
    // std::cout << "\033[32m";
    // std::cout << "Profiling [PeakNode::takeMeasurement()] --- " << std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count() << " us" << std::endl;
    // std::cout << "\033[0m";
}


void PeakNode::populateAScanMessage() {
    ltpa_msg_.header.stamp = ros::Time::now();
    ltpa_msg_.ascans.clear();

    // TODO: Consider matching the peak_ltpa::Ascan and PeakHandler::DofMessage and set appropriate ROS message attributes
    for (auto ascan : ltpa_data_ptr_->ascans) {
        peak_ltpa::Ascan ascan_msg;
        ascan_msg.count = ascan.header.count;
        ascan_msg.test_number = ascan.header.testNo;
        ascan_msg.dof = ascan.header.dof;
        ascan_msg.channel = ascan.header.channel;
        ascan_msg.amplitudes = ascan.amps;
        ltpa_msg_.ascans.push_back(ascan_msg);
    }
}


void PeakNode::populateBScanMessage(const peak_ltpa::Observation& bscan_msg) {
    sensor_msgs::PointCloud2 bscan_cloud;
    bscan_cloud.header.stamp = bscan_msg.header.stamp;
    bscan_cloud.header.frame_id = bscan_msg.header.frame_id;

    // TODO: Move these instantiations out of here
    sensor_msgs::PointCloud2Modifier bscan_cloud_modifier(bscan_cloud);
    bscan_cloud_modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "y", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "z", 1, sensor_msgs::PointField::FLOAT32,               // 32 bits = 4 bytes
        "Amplitudes", 1, sensor_msgs::PointField::INT16         // 16 bits = 2 bytes
        );

    bscan_cloud.height = 1;
    bscan_cloud.width = bscan_msg.ascan_length * bscan_msg.num_ascans;
    bscan_cloud.is_dense = true;
    bscan_cloud.point_step = 14; // 3 fields * 4 bytes per field + 1 field * 2 bytes per field
    bscan_cloud.row_step = bscan_cloud.point_step * bscan_cloud.width;
    bscan_cloud.data.resize(bscan_cloud.row_step);

    // Iterators
    sensor_msgs::PointCloud2Iterator<float>      iterX(bscan_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float>      iterY(bscan_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float>      iterZ(bscan_cloud, "z");
    sensor_msgs::PointCloud2Iterator<int16_t>    iterAmps(bscan_cloud, "Amplitudes");

    //==================================================================================
    // TODO: Check all your units
    //==================================================================================

    // TODO: Confirm digitisation rate is correct
    double dt = (double)1.0 / (double)(bscan_msg.digitisation_rate * (double)1000000.0);              // sec
    double time_in_wedge =       2.0 * bscan_msg.wedge_depth / bscan_msg.vel_wedge / 1000.0;          // sec
    double time_in_couplant =    2.0 * bscan_msg.couplant_depth / bscan_msg.vel_couplant / 1000.0;    // sec
    double time_in_specimen =    2.0 * bscan_msg.specimen_depth / bscan_msg.vel_material / 1000.0;    // sec

    /*
    std::cout << "=====================================" << std::endl;
    std::cout << (double)bscan_msg.digitisation_rate << std::endl; // 100
    std::cout << dt << std::endl;
    std::cout << time_in_wedge << std::endl;
    std::cout << time_in_couplant << std::endl;
    std::cout << time_in_specimen << std::endl;
    std::cout << "=====================================" << std::endl;
    */

    int element_i(0);
    for (const auto& ascan : bscan_msg.ascans) {
        int i = 0;
        for (auto amplitude : ascan.amplitudes) {
            // TODO: If you are going to put a lower bound on the amplitude plotted here would be the place
            *iterX = 0.0;
            *iterY = element_i * bscan_msg.element_pitch * 0.001; // mm to m
            *iterZ = i * bscan_msg.vel_material * dt / (double)2.0;

            // GAT(S) --- GAT <Tn> <Gate Start> <Gate End>
            // Defines search gate start and end positions for the specified test.
            // By default, the gate units are in machine units.
            // A machine unit is defined by the digitisation rate (i.e. 10nSec for 100MHz digitisation).

            // Maybe assume 100 MHz to start with...

            /*
            double t = (double)i * dt;
            double z = 0.0;

            if (t < time_in_wedge) {
                z = t * bscan_msg.vel_wedge;
            } else if (t < time_in_couplant) {
                z = (t - time_in_wedge) * bscan_msg.vel_couplant
                     + bscan_msg.wedge_depth;
            } else if (t < time_in_specimen) {
                z = (t - time_in_wedge - time_in_couplant) * bscan_msg.vel_couplant
                     + bscan_msg.wedge_depth
                     + bscan_msg.couplant_depth;
            }

            *iterZ = z;
            */
            *iterAmps = amplitude; // 20 * log10 ( abs( amplitude / max(amplitude) ) )

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


void PeakNode::run() {
    while (ros::ok() and !*kill_now_) {
        ROS_INFO_STREAM_THROTTLE(600, node_name_ << ": Running");

        if (stream_) {
            takeMeasurement();
        }
        ros::spinOnce();
        rate_.sleep();
    }
}


// Signal handling
bool kill_now = false;


void killNow(int /*unused*/) {
    kill_now = true;
}


auto main(int argc, char** argv) -> int
{
    ros::init(argc, argv, "peak_node");

    PeakNode peak_node(&kill_now);

    signal(SIGTERM, killNow);
    signal(SIGINT, killNow);
    signal(SIGHUP, killNow);

    peak_node.run();

    return 0;
}
