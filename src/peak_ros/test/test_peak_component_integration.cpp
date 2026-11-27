#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "peak_component.hpp"
#include "peak_ros/msg/observation.hpp"
#include "peak_ros/srv/stream_data.hpp"

#include "MockPeakHardware/mock_peak_hardware.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// roller_probe_wing_cover_100_MHz.mps:
//   DOF 4 (16-bit)
//   SWP 1 256 - 316  ->  61 A-scans
//   GATS 1 500 2100  ->  ascan_length = gate_end - gate_start = 1600
static constexpr int EXPECTED_ASCAN_LENGTH = 1600;
static constexpr int EXPECTED_NUM_ASCANS   = 61;


class PeakComponentIntegrationTest : public ::testing::Test {
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

        // --- The component's parameters, as the launch file loads them ---
        const std::vector<rclcpp::Parameter> params = {
            {"settings.acquisition_rate",    20},
            {"settings.peak_address",        std::string("127.0.0.1")},
            {"settings.peak_port",           mock_->port()},
            {"settings.mps_file",            std::string("composite_roller_probe/roller_probe_wing_cover_100_MHz.mps")},
            {"settings.digitisation_rate",   100},
            {"settings.profile",             true},
            {"settings.frame_id",            std::string("ltpa_test")},
            {"settings.reset_sleep_seconds", 0},

            {"settings.tcg.use_tcg",         false},
            {"settings.tcg.amp_factor",      1.0},
            {"settings.tcg.depth_factor",    1.0},
            {"settings.tcg.tcg_limit",       0.7},

            {"settings.gates.gate_front_wall",    0.2},
            {"settings.gates.depth_to_skip",      6.5},
            {"settings.gates.gate_back_wall",     0.7},
            {"settings.gates.max_depth",         17.0},
            {"settings.gates.zero_to_front_wall", true},
            {"settings.gates.show_front_wall",    false},

            {"settings.boundary_conditions.n_elements",            61},
            {"settings.boundary_conditions.element_pitch",          0.8},
            {"settings.boundary_conditions.inter_element_spacing",  0.1},
            {"settings.boundary_conditions.element_width",         10.0},
            {"settings.boundary_conditions.vel_wedge",           2430.0},
            {"settings.boundary_conditions.vel_couplant",        1500.0},
            {"settings.boundary_conditions.vel_material",        2940.0},
            {"settings.boundary_conditions.wedge_angle",            0.0},
            {"settings.boundary_conditions.wedge_depth",            0.0},
            {"settings.boundary_conditions.couplant_depth",         1.0},
            {"settings.boundary_conditions.specimen_depth",        65.0},
        };

        // --- Construct the component, which initialises the hardware before returning ---
        rclcpp::NodeOptions options;
        options.parameter_overrides(params);
        options.arguments({"--ros-args", "-r", "__ns:=/peak"});
        component_ = std::make_shared<peak_namespace::PeakComponent>(options);

        client_node_ = rclcpp::Node::make_shared("peak_component_integration_test");

        executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
        executor_->add_node(component_);
        executor_->add_node(client_node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });
    }

    void TearDown() override {
        executor_->cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
        executor_.reset();
        client_node_.reset();
        component_.reset();
        if (mock_) {
            mock_->stop();
            mock_.reset();
        }
    }

    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr call(const std::string& name,
                                                typename ServiceT::Request::SharedPtr request) {
        auto client = client_node_->create_client<ServiceT>(name);
        if (!client->wait_for_service(5s)) {
            return nullptr;
        }
        // The executor spinning in spin_thread_ completes the future
        auto future = client->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready) {
            return nullptr;
        }
        return future.get();
    }

    bool callSingleMeasurement() {
        auto response = call<std_srvs::srv::Trigger>("/peak/take_single_measurement",
                                                     std::make_shared<std_srvs::srv::Trigger::Request>());
        return response && response->success;
    }

    bool callStream(bool enable) {
        auto request = std::make_shared<peak_ros::srv::StreamData::Request>();
        request->stream_data = enable;
        auto response = call<peak_ros::srv::StreamData>("/peak/stream_data", request);
        return response && response->success;
    }

    // Waits for the subscriber to match the component's publisher
    template <typename SubscriptionT>
    void waitForPublisher(const SubscriptionT& subscription) {
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(50ms);
        }
    }

    // Waits until done() holds or 5 s pass
    template <typename Predicate>
    void waitFor(Predicate done) {
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!done() && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(50ms);
        }
    }

    std::unique_ptr<MockPeakHardware>                          mock_;
    std::shared_ptr<peak_namespace::PeakComponent>             component_;
    rclcpp::Node::SharedPtr                                    client_node_;
    std::unique_ptr<rclcpp::executors::MultiThreadedExecutor>  executor_;
    std::thread                                                spin_thread_;
};


// ---------------------------------------------------------------------------
// Test 1: Single measurement round-trip
// ---------------------------------------------------------------------------
TEST_F(PeakComponentIntegrationTest, SingleMeasurementRoundTrip) {
    std::mutex mtx;
    peak_ros::msg::Observation::SharedPtr received;
    auto sub = client_node_->create_subscription<peak_ros::msg::Observation>(
        "/peak/a_scans", 10,
        [&](peak_ros::msg::Observation::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx);
            received = msg;
        });
    waitForPublisher(sub);

    ASSERT_TRUE(callSingleMeasurement()) << "Service call failed";

    waitFor([&]() { std::lock_guard<std::mutex> lock(mtx); return received != nullptr; });

    std::lock_guard<std::mutex> lock(mtx);
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
TEST_F(PeakComponentIntegrationTest, BScanPointCloudStructure) {
    std::mutex mtx;
    sensor_msgs::msg::PointCloud2::SharedPtr received;
    auto sub = client_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/peak/b_scan", 10,
        [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx);
            received = msg;
        });
    waitForPublisher(sub);

    ASSERT_TRUE(callSingleMeasurement());

    waitFor([&]() { std::lock_guard<std::mutex> lock(mtx); return received != nullptr; });

    std::lock_guard<std::mutex> lock(mtx);
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
TEST_F(PeakComponentIntegrationTest, GatedBScanPointCloudStructure) {
    std::mutex mtx;
    sensor_msgs::msg::PointCloud2::SharedPtr received;
    auto sub = client_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/peak/gated_b_scan", 10,
        [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx);
            received = msg;
        });
    waitForPublisher(sub);

    ASSERT_TRUE(callSingleMeasurement());

    waitFor([&]() { std::lock_guard<std::mutex> lock(mtx); return received != nullptr; });

    std::lock_guard<std::mutex> lock(mtx);
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
TEST_F(PeakComponentIntegrationTest, StreamingProducesMultipleMessages) {
    std::mutex mtx;
    std::vector<peak_ros::msg::Observation::SharedPtr> messages;
    auto sub = client_node_->create_subscription<peak_ros::msg::Observation>(
        "/peak/a_scans", 10,
        [&](peak_ros::msg::Observation::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mtx);
            messages.push_back(msg);
        });
    waitForPublisher(sub);

    ASSERT_TRUE(callStream(true)) << "Failed to start streaming";

    // Collect messages for ~500ms
    std::this_thread::sleep_for(500ms);

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
TEST_F(PeakComponentIntegrationTest, MockServerCounters) {
    // After component initialisation, a reset should have been sent
    EXPECT_EQ(mock_->resetCount(), 1)
        << "Expected exactly 1 reset during init";

    EXPECT_GT(mock_->configLinesCount(), 0)
        << "Expected config lines to have been sent during MPS configuration";

    // Now request a single measurement
    ASSERT_TRUE(callSingleMeasurement());

    // Allow processing
    std::this_thread::sleep_for(300ms);

    EXPECT_EQ(mock_->dataRequestCount(), 1)
        << "Expected exactly 1 data request after single measurement";
}


// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    int result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}
