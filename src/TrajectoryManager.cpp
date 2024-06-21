
#include "TrajectoryManager.hpp"

#include <memory>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

TrajectoryManager::TrajectoryManager(
    int argc, char *argv[], const std::string &node_name,
    const std::string &locations_frame, const std::string &locations_csv,
    const std::string &gensim_model_file,
    const std::string &last_route_publisher_topic,
    const std::string &location_update_topic, const std::string &go_to_service,
    const std::string &get_current_location_service,
    const std::string &get_route_for_object_service,
    const std::string &go_to_location_service,
    const std::string &add_current_location_service,
    const std::string &closest_location_to_position_service
) :
    locations_manager(locations_csv), gensim_model(gensim_model_file) {
    // ROS
    ros::init(argc, argv, node_name);
    this->nh = std::make_unique<ros::NodeHandle>();

    // ROS TF
    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(100));
    this->tf_listener =
        std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer);

    ros::Duration(1).sleep();

    // Publishers
    this->last_route_pub =
        this->nh->advertise<nav_msgs::Path>(last_route_publisher_topic, 10);

    // Subscribers
    this->nh->subscribe(
        location_update_topic,
        1,
        &TrajectoryManager::update_location_callback,
        this
    );

    ROS_INFO("Waiting for position update...");
    ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(
        location_update_topic
    );
    ROS_INFO("Waiting for position update... Done!");

    // Services: Robot Manager
    this->go_to_service =
        nh->serviceClient<trajectory_manager::GoTo>(go_to_service);

    // Services
    this->nh->advertiseService(
        get_current_location_service,
        &TrajectoryManager::get_current_location,
        this
    );
    this->nh->advertiseService(
        get_route_for_object_service,
        &TrajectoryManager::get_route_for_object,
        this
    );
    this->nh->advertiseService(
        go_to_location_service, &TrajectoryManager::go_to_location, this
    );
    this->nh->advertiseService(
        add_current_location_service,
        &TrajectoryManager::add_current_location,
        this
    );
    this->nh->advertiseService(
        closest_location_to_position_service,
        &TrajectoryManager::closest_location_to_position,
        this
    );
}

bool TrajectoryManager::add_current_location(
    trajectory_manager::AddCurrentLocation::Request &req,
    trajectory_manager::AddCurrentLocation::Response &res
) {
    // Make sure current location has been updated at least once
    if (this->current_location == std::nullopt) {
        res.success = false;
        ROS_ERROR("No current location set");
        return false;
    }

    // Update locations csv file
    std::string location_name = req.location_name;
    this->locations_manager.add_location(
        Location(location_name, this->current_location.value())
    );

    // Send response
    res.success = true;
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = this->current_location.value();
    res.pose = pose_stamped;
    return true;
}

void TrajectoryManager::update_location_callback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose
) {
    // Update current location
    this->current_location = pose->pose;
}

bool TrajectoryManager::get_current_location(
    trajectory_manager::GetCurrentLocation::Request &req,
    trajectory_manager::GetCurrentLocation::Response &res
) {
    // Make sure current location has been updated at least once
    if (this->current_location == std::nullopt) {
        ROS_ERROR("No current location set");
        return false;
    }

    // Send response
    geometry_msgs::PoseWithCovarianceStamped pose_stamped;
    pose_stamped.pose = this->current_location.value();
    res.pose = pose_stamped;
    return true;
}

bool TrajectoryManager::get_route_for_object(
    trajectory_manager::GetRouteForObject::Request &req,
    trajectory_manager::GetRouteForObject::Response &res
) {
    const std::string object_class = req.object_class;

    // Get similarities between object class and locations
    std::vector<std::pair<double, Location>> semantic_similarity =
        this->locations_manager.get_semantic_similarity(object_class);

    // Sort the locations by similarity
    std::sort(
        semantic_similarity.begin(),
        semantic_similarity.end(),
        [](const auto &a, const auto &b) { return a.first > b.first; }
    );

    // Create a queue of locations to visit
    nav_msgs::Path path;
    path.header.frame_id = this->locations_frame;
    for (const auto &[similarity, location] : semantic_similarity) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = this->locations_frame;
        pose.pose = location.to_pose();
        path.poses.push_back(pose);
    }

    // Send response
    res.path = path;

    // Publish the last route
    this->last_route_pub.publish(path);

    return true;
}

bool TrajectoryManager::go_to_location(
    trajectory_manager::GoToLocation::Request &req,
    trajectory_manager::GoToLocation::Response &res
) {
    // Get location
    const std::string location_name = req.location;
    const auto location = this->locations_manager.get_location(location_name);

    // Make sure location exists
    if (location == std::nullopt) {
        ROS_ERROR("Location %s not found", location_name.c_str());
        res.success = false;
        return false;
    }

    // Create a GoTo service request
    trajectory_manager::GoTo::Request go_to_req;
    go_to_req.goal.header.frame_id = this->locations_frame;
    go_to_req.goal.pose = location.value().to_pose();

    // Call the GoTo service
    trajectory_manager::GoTo::Response go_to_res;
    this->go_to_service.call(go_to_req, go_to_res);
    res.success = true;

    return true;
}

bool TrajectoryManager::closest_location_to_position(
    trajectory_manager::ClosestLocationToPosition::Request &req,
    trajectory_manager::ClosestLocationToPosition::Response &res
) {
    // Get position
    auto target_point = this->transform_to_tf(
        req.pose, this->locations_frame, ros::Time::now()
    );

    // Get closest location
    const std::optional<Location> closest_location =
        this->locations_manager.get_closest_location(target_point);

    // Make sure location exists
    if (closest_location == std::nullopt) {
        ROS_ERROR("No locations found");
        res.success = false;
        return false;
    }

    // Send response
    res.success = true;
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose = closest_location.value().to_pose();
    res.pose = pose_stamped;

    return true;
}

geometry_msgs::Pose TrajectoryManager::transform_to_tf(
    const geometry_msgs::PoseStamped &pose, const std::string &to_frame,
    const ros::Time &time
) {
    // Lookup transform
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = this->tf_buffer->lookupTransform(
            to_frame, pose.header.frame_id, time
        );
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return geometry_msgs::Pose();
    }

    // Transform pose
    geometry_msgs::Pose transformed_pose;
    tf2::doTransform(pose.pose, transformed_pose, transform_stamped);

    // Return transformed pose
    return transformed_pose;
}

int main(int argc, char **argv) {
    TrajectoryManager tm(argc, argv);
    ros::spin();
    return 0;
}