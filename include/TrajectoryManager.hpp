#pragma once

#include "LocationManager.hpp"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Path.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_manager/AddCurrentLocation.h"
#include "trajectory_manager/ClosestLocationToPosition.h"
#include "trajectory_manager/GetCurrentLocation.h"
#include "trajectory_manager/GetRouteForObject.h"
#include "trajectory_manager/GoTo.h"
#include "trajectory_manager/GoToLocation.h"

class TrajectoryManager {
private:
    // Locations
    std::optional<geometry_msgs::PoseWithCovariance> current_location =
        std::nullopt;
    LocationManager locations_manager;
    std::string locations_frame;

    // ROS
    std::unique_ptr<ros::NodeHandle> nh = nullptr;

    // ROS TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = nullptr;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener = nullptr;

    // ROS Publishers
    ros::Publisher last_route_pub;

    // ROS Services
    ros::ServiceServer get_current_location_service;
    ros::ServiceServer get_route_for_object_service;
    ros::ServiceServer go_to_location_service;
    ros::ServiceServer add_current_location_service;
    ros::ServiceServer closest_location_to_position_service;

    // ROS Service Proxy
    ros::ServiceClient go_to_service;

public:
    TrajectoryManager(
        int argc, char *argv[],
        const std::string &node_name = "trajectory_manager",
        const std::string &locations_frame = "map",
        const std::string &locations_csv = "data/locations5.csv",
        const std::string &gensim_model_file = "data/gensim/model.bin",
        const std::string &last_route_publisher_topic =
            "/trajectory_manager/last_route",
        const std::string &location_update_topic = "/amcl_pose",
        const std::string &go_to_service = "/robot_manager/go_to_service",
        const std::string &get_current_location_service =
            "/trajectory_manager/get_current_location",
        const std::string &get_route_for_object_service =
            "/trajectory_manager/get_route_for_object",
        const std::string &go_to_location_service =
            "/trajectory_manager/go_to_location",
        const std::string &add_current_location_service =
            "/trajectory_manager/add_current_location",
        const std::string &closest_location_to_position_service =
            "/trajectory_manager/closest_location_to_position"
    );

    bool add_current_location(
        trajectory_manager::AddCurrentLocation::Request &req,
        trajectory_manager::AddCurrentLocation::Response &res
    );

    void update_location_callback(
        const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose
    );

    bool get_current_location(
        trajectory_manager::GetCurrentLocation::Request &req,
        trajectory_manager::GetCurrentLocation::Response &res
    );

    bool get_route_for_object(
        trajectory_manager::GetRouteForObject::Request &req,
        trajectory_manager::GetRouteForObject::Response &res
    );

    bool go_to_location(
        trajectory_manager::GoToLocation::Request &req,
        trajectory_manager::GoToLocation::Response &res
    );

    bool closest_location_to_position(
        trajectory_manager::ClosestLocationToPosition::Request &req,
        trajectory_manager::ClosestLocationToPosition::Response &res
    );

    geometry_msgs::Pose transform_to_tf(
        const geometry_msgs::PoseStamped &pose, const std::string &to_frame,
        const ros::Time &time = ros::Time(0)
    );
};