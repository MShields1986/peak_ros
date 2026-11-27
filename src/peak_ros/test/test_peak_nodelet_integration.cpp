#include <gtest/gtest.h>

#include <ros/ros.h>
#include <nodelet/loader.h>

#include <peak_ros/Observation.h>
#include <peak_ros/TakeSingleMeasurement.h>
#include <peak_ros/StreamData.h>
#include <sensor_msgs/PointCloud2.h>

#include "MockPeakHardware/mock_peak_hardware.h"

#include <memory>
#include <mutex>
#include <string>
#include <vector>

// roller_probe_wing_cover_100_MHz.mps:
//   DOF 4 (16-bit)
//   SWP 1 256 - 316  ->  61 A-scans
//   GATS 1 500 2100  ->  ascan_length = gate_end - gate_start = 1600
static constexpr int EXPECTED_ASCAN_LENGTH = 1600;
static constexpr int EXPECTED_NUM_ASCANS   = 61;


class PeakNodeletIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // --- Start mock hardware ---
        MockPeakHardware::Config cfg;
        cfg.port             = 0;
        cfg.dof              = 4;        // 16-bit (matches DOF 4 in MPS)
        cfg.ascan_length     = EXPECTED_ASCAN_LENGTH;
        cfg.num_a_scans      = EXPECTED_NUM_ASCANS;
        cfg.actual_dig_rate  = 100;
        cfg.default_dig_rate = 100;

        mock_ = std::make_unique<MockPeakHardware>(cfg);
        mock_->start();

        // --- Set all required ROS params ---
        ros::NodeHandle nh;
        std::string ns = "/peak/settings";

        nh.setParam(ns + "/acquisition_rate",    20);
        nh.setParam(ns + "/peak_address",        std::string("127.0.0.1"));
        nh.setParam(ns + "/peak_port",           mock_->port());
        nh.setParam(ns + "/mps_file",            std::string("composite_roller_probe/roller_probe_wing_cover_100_MHz.mps"));
        nh.setParam(ns + "/digitisation_rate",   100);
        nh.setParam(ns + "/profile",             true);
        nh.setParam(ns + "/frame_id",            std::string("ltpa_test"));
        nh.setParam(ns + "/reset_sleep_seconds", 0);

        nh.setParam(ns + "/tcg/use_tcg",       false);
        nh.setParam(ns + "/tcg/amp_factor",     1.0);
        nh.setParam(ns + "/tcg/depth_factor",   1.0);
        nh.setParam(ns + "/tcg/tcg_limit",      0.7);

        nh.setParam(ns + "/gates/gate_front_wall",    0.2);
        nh.setParam(ns + "/gates/depth_to_skip",      6.5);
        nh.setParam(ns + "/gates/gate_back_wall",     0.7);
        nh.setParam(ns + "/gates/max_depth",         17.0);
        nh.setParam(ns + "/gates/zero_to_front_wall", true);
        nh.setParam(ns + "/gates/show_front_wall",    false);

        nh.setParam(ns + "/boundary_conditions/n_elements",            61);
        nh.setParam(ns + "/boundary_conditions/element_pitch",          0.8);
        nh.setParam(ns + "/boundary_conditions/inter_element_spacing",  0.1);
        nh.setParam(ns + "/boundary_conditions/element_width",         10.0);
        nh.setParam(ns + "/boundary_conditions/vel_wedge",           2430.0);
        nh.setParam(ns + "/boundary_conditions/vel_couplant",        1500.0);
        nh.setParam(ns + "/boundary_conditions/vel_material",        2940.0);
        nh.setParam(ns + "/boundary_conditions/wedge_angle",            0.0);
        nh.setParam(ns + "/boundary_conditions/wedge_depth",            0.0);
        nh.setParam(ns + "/boundary_conditions/couplant_depth",         1.0);
        nh.setParam(ns + "/boundary_conditions/specimen_depth",        65.0);

        // --- Load nodelet ---
        loader_ = std::make_unique<nodelet::Loader>();

        std::map<std::string, std::string> remappings;
        std::vector<std::string> argv;

        bool loaded = loader_->load(
            "/peak",                              // name
            "peak_ros/peak_nodelet",              // type
            remappings,
            argv
        );
        ASSERT_TRUE(loaded) << "Failed to load PeakNodelet via nodelet::Loader";

        // Allow time for nodelet initialisation and publisher/subscriber connections
        ros::Duration(0.5).sleep();
    }

    void TearDown() override {
        if (loader_) {
            loader_->unload("/peak");
            loader_.reset();
        }
        if (mock_) {
            mock_->stop();
            mock_.reset();
        }
        // Clear all params set during the test
        ros::NodeHandle nh;
        nh.deleteParam("/peak");
    }

    bool callSingleMeasurement() {
        ros::NodeHandle nh;
        ros::ServiceClient client =
            nh.serviceClient<peak_ros::TakeSingleMeasurement>("/peak/take_single_measurement");

        if (!client.waitForExistence(ros::Duration(5.0))) {
            return false;
        }

        peak_ros::TakeSingleMeasurement srv;
        srv.request.take_single_measurement = true;
        return client.call(srv) && srv.response.success;
    }

    bool callStream(bool enable) {
        ros::NodeHandle nh;
        ros::ServiceClient client =
            nh.serviceClient<peak_ros::StreamData>("/peak/stream_data");

        if (!client.waitForExistence(ros::Duration(5.0))) {
            return false;
        }

        peak_ros::StreamData srv;
        srv.request.stream_data = enable;
        return client.call(srv) && srv.response.success;
    }

    std::unique_ptr<MockPeakHardware>  mock_;
    std::unique_ptr<nodelet::Loader>   loader_;
};


// ---------------------------------------------------------------------------
// Test 1: Single measurement round-trip
// ---------------------------------------------------------------------------
TEST_F(PeakNodeletIntegrationTest, SingleMeasurementRoundTrip) {
    ros::NodeHandle nh;

    peak_ros::Observation::ConstPtr received;
    auto sub = nh.subscribe<peak_ros::Observation>(
        "/peak/a_scans", 1,
        [&received](const peak_ros::Observation::ConstPtr& msg) {
            received = msg;
        }
    );

    // Let subscriber connect
    ros::Duration(0.3).sleep();

    ASSERT_TRUE(callSingleMeasurement()) << "Service call failed";

    // Wait for message
    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    while (!received && ros::Time::now() < deadline) {
        ros::Duration(0.05).sleep();
    }

    ASSERT_TRUE(received) << "No Observation message received on /peak/a_scans";

    EXPECT_EQ(received->num_ascans, EXPECTED_NUM_ASCANS);
    EXPECT_EQ(received->ascan_length, EXPECTED_ASCAN_LENGTH);
    ASSERT_EQ(static_cast<int>(received->ascans.size()), EXPECTED_NUM_ASCANS);

    // Verify first ascan amplitudes match mock pattern:
    // DOF 4: raw = 32768 + ((ascan_index*7 + i) % 1000), parsed = raw - 32768
    // For ascan_index=0: amps[i] == i % 1000
    const auto& first_ascan = received->ascans[0];
    ASSERT_EQ(static_cast<int>(first_ascan.amplitudes.size()), EXPECTED_ASCAN_LENGTH);
    for (int i = 0; i < EXPECTED_ASCAN_LENGTH; ++i) {
        EXPECT_EQ(first_ascan.amplitudes[i], i % 1000)
            << "Mismatch at sample " << i;
    }
}


// ---------------------------------------------------------------------------
// Test 2: B-scan PointCloud2 structure
// ---------------------------------------------------------------------------
TEST_F(PeakNodeletIntegrationTest, BScanPointCloudStructure) {
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2::ConstPtr received;
    auto sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/peak/b_scan", 1,
        [&received](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            received = msg;
        }
    );

    ros::Duration(0.3).sleep();

    ASSERT_TRUE(callSingleMeasurement());

    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    while (!received && ros::Time::now() < deadline) {
        ros::Duration(0.05).sleep();
    }

    ASSERT_TRUE(received) << "No PointCloud2 message received on /peak/b_scan";

    EXPECT_EQ(received->width,  static_cast<uint32_t>(EXPECTED_ASCAN_LENGTH * EXPECTED_NUM_ASCANS));
    EXPECT_EQ(received->height, 1u);
    EXPECT_EQ(received->point_step, 16u);  // 4 fields * 4 bytes

    ASSERT_EQ(received->fields.size(), 4u);
    EXPECT_EQ(received->fields[0].name, "x");
    EXPECT_EQ(received->fields[1].name, "y");
    EXPECT_EQ(received->fields[2].name, "z");
    EXPECT_EQ(received->fields[3].name, "Amplitudes");
}


// ---------------------------------------------------------------------------
// Test 3: Gated B-scan PointCloud2 structure
// ---------------------------------------------------------------------------
TEST_F(PeakNodeletIntegrationTest, GatedBScanPointCloudStructure) {
    ros::NodeHandle nh;

    sensor_msgs::PointCloud2::ConstPtr received;
    auto sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/peak/gated_b_scan", 1,
        [&received](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            received = msg;
        }
    );

    ros::Duration(0.3).sleep();

    ASSERT_TRUE(callSingleMeasurement());

    ros::Time deadline = ros::Time::now() + ros::Duration(5.0);
    while (!received && ros::Time::now() < deadline) {
        ros::Duration(0.05).sleep();
    }

    ASSERT_TRUE(received) << "No PointCloud2 message received on /peak/gated_b_scan";

    EXPECT_EQ(received->width,  static_cast<uint32_t>(EXPECTED_ASCAN_LENGTH * EXPECTED_NUM_ASCANS));
    EXPECT_EQ(received->height, 1u);
    EXPECT_EQ(received->point_step, 20u);  // 5 fields * 4 bytes

    ASSERT_EQ(received->fields.size(), 5u);
    EXPECT_EQ(received->fields[0].name, "x");
    EXPECT_EQ(received->fields[1].name, "y");
    EXPECT_EQ(received->fields[2].name, "z");
    EXPECT_EQ(received->fields[3].name, "Amplitudes");
    EXPECT_EQ(received->fields[4].name, "TimeofFlight");
}


// ---------------------------------------------------------------------------
// Test 4: Streaming produces multiple messages
// ---------------------------------------------------------------------------
TEST_F(PeakNodeletIntegrationTest, StreamingProducesMultipleMessages) {
    ros::NodeHandle nh;

    std::mutex mtx;
    std::vector<peak_ros::Observation::ConstPtr> messages;

    auto sub = nh.subscribe<peak_ros::Observation>(
        "/peak/a_scans", 10,
        [&](const peak_ros::Observation::ConstPtr& msg) {
            std::lock_guard<std::mutex> lock(mtx);
            messages.push_back(msg);
        }
    );

    ros::Duration(0.3).sleep();

    ASSERT_TRUE(callStream(true)) << "Failed to start streaming";

    // Collect messages for ~500ms
    ros::Time deadline = ros::Time::now() + ros::Duration(0.5);
    while (ros::Time::now() < deadline) {
        ros::Duration(0.05).sleep();
    }

    ASSERT_TRUE(callStream(false)) << "Failed to stop streaming";

    std::lock_guard<std::mutex> lock(mtx);
    EXPECT_GE(static_cast<int>(messages.size()), 2)
        << "Expected at least 2 streamed Observation messages, got " << messages.size();

    // Verify structure of each received message
    for (const auto& msg : messages) {
        EXPECT_EQ(msg->num_ascans, EXPECTED_NUM_ASCANS);
        EXPECT_EQ(msg->ascan_length, EXPECTED_ASCAN_LENGTH);
    }
}


// ---------------------------------------------------------------------------
// Test 5: Mock server counters
// ---------------------------------------------------------------------------
TEST_F(PeakNodeletIntegrationTest, MockServerCounters) {
    // After nodelet initialisation, a reset should have been sent
    EXPECT_EQ(mock_->resetCount(), 1)
        << "Expected exactly 1 reset during init";

    EXPECT_GT(mock_->configLinesCount(), 0)
        << "Expected config lines to have been sent during MPS configuration";

    // Now request a single measurement
    ASSERT_TRUE(callSingleMeasurement());

    // Allow processing
    ros::Duration(0.3).sleep();

    EXPECT_EQ(mock_->dataRequestCount(), 1)
        << "Expected exactly 1 data request after single measurement";
}


// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "peak_nodelet_integration_test");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    int result = RUN_ALL_TESTS();

    spinner.stop();
    ros::shutdown();
    return result;
}
