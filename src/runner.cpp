// runner.cpp

#include "runner.h"

float Vector3::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

Runner::Runner(tf2_ros::Buffer& tf_buffer)
    : Node("driver_node", rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)), tf_buffer_(tf_buffer) {

    try {
        // Fetch parameters
        fetchParameters();

        // Initialize data members and publishers
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

        // Initialize other member variables and perform setup here
        twist_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
        mode_ = 0;

        // Update dynamic transformations
        update_transform_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Adjust time interval as needed
            std::bind(&Runner::updateTransform, this));

    } catch (const std::exception& e) {
        std::cerr << "Exception caught in Runner constructor: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown exception caught in Runner constructor" << std::endl;
    }
}

void Runner::fetchParameters() {
    rclcpp::Parameter aruco0, aruco1, aruco2;
    aruco0 = this->get_parameter("aruco_marker_0");
    aruco1 = this->get_parameter("aruco_marker_1");
    aruco2 = this->get_parameter("aruco_marker_2");

    params_.insert(std::pair<std::string, std::string>("aruco_marker_0", aruco0.value_to_string().c_str()));
    params_.insert(std::pair<std::string, std::string>("aruco_marker_1", aruco1.value_to_string().c_str()));
    params_.insert(std::pair<std::string, std::string>("aruco_marker_2", aruco2.value_to_string().c_str()));
}

void Runner::arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {

    if (!msg->marker_ids.empty()) {
        Vector3 min_position = msg->poses[0];
        int min_index = 0;

        // Iterate through the array starting from the second pose to find the closest aruco
        for (int i = 1; i < int(msg->poses.size()); ++i) {
            const Vector3& position = msg->poses[i];
            // Compare the position vector of the current pose with the minimum position vector
            if (position.magnitude() < min_position.magnitude()) {
                // Update the minimum position vector and its index
                min_position = msg->poses[i];
                min_index = i;
            }
        }

        markerID = msg->marker_ids[min_index];

        geometry_msgs::msg::PoseStamped aruco_base_pose;
        geometry_msgs::msg::PoseStamped aruco_odom_pose;
        geometry_msgs::msg::PoseStamped aruco_pose;
        aruco_pose.header.frame_id = source_frame1;
        aruco_pose.pose = msg->poses[min_index];

        tf2::doTransform(aruco_pose, aruco_base_pose, cam_to_base_);
        tf2::doTransform(aruco_pose, aruco_odom_pose, transformStamped1);

        auto pos_t_base = aruco_base_pose.pose;
        auto pos_t_odom = aruco_odom_pose.pose;

        arucoPos_ = {pos_t_base.position.x, pos_t_base.position.y, pos_t_base.position.z};
        aruco_odom_ = {pos_t_odom.position.x, pos_t_odom.position.y, pos_t_odom.position.z};
    }
    body();
}

void Runner::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odomPos_ = {msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
    body();
}

void Runner::batteryCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (!msg->part_poses.empty()) {
        auto color = msg->part_poses[0].part.color;

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("Color"), "Color = " << std::to_string(color) << " ");

        geometry_msgs::msg::PoseStamped battery_pose;
        geometry_msgs::msg::PoseStamped battery_odom_pose;

        geometry_msgs::msg::Pose pose;

        battery_pose.header.frame_id = source_frame1;
        battery_pose.pose = msg->part_poses[0].pose;

        // Extra (questionable) Y rotation to 'fix' the frame problem
        geometry_msgs::msg::TransformStamped testT = transformStamped2;
        tf2::Quaternion q_test;
        tf2::Quaternion q_orig;
        tf2::fromMsg(transformStamped2.transform.rotation, q_orig);
        q_test.setRPY(0.0, -0.4, 0.0);
        q_orig = q_orig * q_test;
        testT.transform.rotation = tf2::toMsg(q_orig);


        tf2::doTransform(battery_pose, battery_odom_pose, testT);

        battery_odom_[color] = battery_odom_pose.pose;

        pose = battery_odom_pose.pose;

        tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    }
}

void Runner::driveForward() {
    twist_msg_->linear.x = 0.2;
    twist_msg_->angular.z = arucoPos_[1] * 0.2;
    publisher_->publish(*twist_msg_);
}

void Runner::turnRobot() {
    twist_msg_->linear.x = 0.0;
    twist_msg_->angular.z = (-3.14159 / 20) * mode_;
    publisher_->publish(*twist_msg_);
}

void Runner::updateTransform() {
    try {
        transformStamped1 = tf_buffer_.lookupTransform(odom_frame, source_frame1, tf2::TimePointZero, tf2::durationFromSec(1.0));
        transformStamped2 = tf_buffer_.lookupTransform(odom_frame, source_frame2, tf2::TimePointZero, tf2::durationFromSec(1.0));
        cam_to_base_ = tf_buffer_.lookupTransform(target_frame, source_frame1, tf2::TimePointZero, tf2::durationFromSec(4.0));
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Transform lookup failed: " << ex.what());
    }
}

void Runner::body() {
    // Main code

    if (odomPos_.empty() || arucoPos_.empty() || mode_ == 3) return;

    auto dist = Distance(aruco_odom_, odomPos_);

    // RCLCPP_INFO_STREAM(rclcpp::get_logger("driver_node_threshold_0.8"), "Distance " << dist << "\n");

    if (dist > 0.8 && mode_ == 0) {
        // Keep driving
        driveForward();
    }

    else {
        if (mode_ == 0) {
            turn_time_ = my_clock_.now();
            std::string direction = params_["aruco_marker_" + std::to_string(markerID)];

            if (direction == "right_90") {
                mode_ = 1;
            }
            if (direction == "left_90") {
                mode_ = -1;
            }
            if (direction == "end") {
                mode_ = 3;
                twist_msg_->linear.x = 0;
                twist_msg_->angular.z = 0;
                publisher_->publish(*twist_msg_);

                for (const auto& pair : battery_odom_) {
                    int key = pair.first;
                    const auto& pose = pair.second;
                    std::string color;

                    tf2::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                    if (key == 1) {
                        color = "Green";
                    } else if (key == 2) {
                        color = "Blue";
                    } else if (key == 3) {
                        color = "Orange";
                    }

                    RCLCPP_INFO_STREAM(rclcpp::get_logger("driver_node"), color << " battery detected at xyz=[" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
                    << "] rpy=[" << roll << ", " << pitch << ", " << yaw << "]");

                }
                return;
            }
        }

        turnRobot();

        if ((mode_ == 1 || mode_ == -1) && my_clock_.now() - turn_time_ >= rclcpp::Duration::from_seconds(10)) {
            mode_ = 0;
            twist_msg_->angular.z = 0.0;
            publisher_->publish(*twist_msg_);
        }
    }
}

double Runner::Distance(const std::vector<double>& list1, const std::vector<double>& list2) {
    auto dist = sqrt(pow(list1[0] - list2[0], 2) + pow(list1[1] - list2[1], 2));
    return dist;
}


// The main function of the program
int main(int argc, char* argv[]) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a buffer for storing TF2 transforms
    tf2_ros::Buffer tf_buffer(std::make_shared<rclcpp::Clock>());

    // Create an instance of the Runner class, passing the TF buffer
    auto node = std::make_shared<Runner>(tf_buffer);

    // Set up Quality of Service (QoS) settings for the subscriptions
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // Create a subscription to the "/aruco_markers" topic
    auto subscription_aruco_ = node->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/aruco_markers", qos, [node](const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
            node->arucoMarkerCallback(msg);
        });

    // Create a subscription to the "/odom" topic
    auto subscription_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [node](const nav_msgs::msg::Odometry::SharedPtr msg) {
            node->odomCallback(msg);
        });

    // Create a subscription to the "/mage/advanced_logical_camera/image" topic
    auto subscription_battery_ = node->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/advanced_logical_camera/image", qos, [node](const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
            node->batteryCallback(msg);
        });

    // Spin the node, which starts the event loop and processes callbacks
    rclcpp::spin(node);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    // Return 0 to indicate successful execution
    return 0;
}
