
#include "Location.hpp"

Location::Location(
    const std::string &name, double x, double y, double theta, double w
) :
    name(name), x(x), y(y), theta(theta), w(w) {}

Location::Location(
    const std::string &name, const geometry_msgs::PoseWithCovariance &pose
) :
    name(name),
    x(pose.pose.position.x),
    y(pose.pose.position.y),
    theta(pose.pose.orientation.z),
    w(pose.pose.orientation.w) {}

Location::Location(const Location &location) :
    name(location.name),
    x(location.x),
    y(location.y),
    theta(location.theta),
    w(location.w) {}

geometry_msgs::Pose Location::to_pose() const {
    geometry_msgs::Pose pose;
    pose.position.x = this->x;
    pose.position.y = this->y;
    pose.orientation.z = this->theta;
    pose.orientation.w = this->w;
    return pose;
}