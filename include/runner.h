// runner.h

#ifndef RUNNER_H
#define RUNNER_H

#include <cstdio>
#include <cmath>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

struct Vector3 {
    float x; ///< X component of the vector.
    float y; ///< Y component of the vector.
    float z; ///< Z component of the vector.

    /**
     * @brief Constructor that initializes a Vector3 object from a Pose message.
     *
     * The Vector3 is initialized with the position part of the Pose.
     * @param pose A Pose message from which the position is extracted.
     */
    Vector3(const geometry_msgs::msg::Pose& pose) : x(pose.position.x), y(pose.position.y), z(pose.position.z) {};

    /**
     * @brief Calculates the magnitude of the vector.
     *
     * @return float The magnitude of the vector.
     */
    float magnitude() const;
};


/**
 * @brief The Runner class is responsible for controlling the robot's movement and processing sensor data.
 */
class Runner : public rclcpp::Node {
public:
    /**
     * @brief Constructor for the Runner class.
     *
     * @param tf_buffer A reference to a tf2_ros::Buffer object.
     */
    Runner(tf2_ros::Buffer& tf_buffer);

    /**
     * @brief Callback function for Aruco marker detection.
     *
     * @param msg Shared pointer to the received ArucoMarkers message.
     */
    void arucoMarkerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Callback function for odometry data.
     *
     * @param msg Shared pointer to the received Odometry message.
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function for battery data.
     *
     * @param msg Shared pointer to the received AdvancedLogicalCameraImage message.
     */
    void batteryCallback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

private:
    /**
     * @brief Calculates the Euclidean distance between two points.
     *
     * @param vec1 First vector.
     * @param vec2 Second vector.
     * @return double Euclidean distance between vec1 and vec2.
     */
    double Distance(const std::vector<double>& vec1, const std::vector<double>& vec2);

    /**
     * @brief Commands the robot to drive forward.
     */
    void driveForward();

    /**
     * @brief Commands the robot to turn.
     */
    void turnRobot();

    /**
     * @brief Updates the transformation matrices based on current sensor data.
     */
    void updateTransform();

    /**
     * @brief Main processing body of the class.
     */
    void body();

    /**
     * @brief Fetches and processes parameters from the parameter server.
     */
    void fetchParameters();

    // Member variables with brief descriptions
    const std::string source_frame1 = "camera_rgb_optical_frame";
    const std::string source_frame2 = "logical_camera_link";
    const std::string target_frame = "base_footprint";
    const std::string odom_frame = "odom";


    // Publisher for sending Twist messages
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Buffer for storing and managing transforms
    tf2_ros::Buffer& tf_buffer_;

    // Listener for receiving and processing transforms
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Timer for updating the transformation matrices
    rclcpp::TimerBase::SharedPtr update_transform_timer_;

    // Map for storing parameters
    std::map<std::string, std::string> params_;

    // Transform between camera and base_footprint frames
    geometry_msgs::msg::TransformStamped cam_to_base_;

    // TransformStamped objects for storing transforms
    geometry_msgs::msg::TransformStamped transformStamped1;
    geometry_msgs::msg::TransformStamped transformStamped2;

    // Vectors for storing position data
    std::vector<double> odomPos_{};
    std::vector<double> arucoPos_{};
    std::vector<double> aruco_odom_{};

    // Map for storing battery and odom transforms
    std::map<int, geometry_msgs::msg::Pose> battery_odom_;

    // Marker ID for Aruco marker detection
    int markerID;

    // Shared pointer for Twist message
    std::shared_ptr<geometry_msgs::msg::Twist> twist_msg_;

    // Time for tracking turn duration
    rclcpp::Time turn_time_;

    // Clock for time-related operations
    rclcpp::Clock my_clock_;

    // Mode for controlling robot behavior
    int mode_;

};

#endif // RUNNER_H
