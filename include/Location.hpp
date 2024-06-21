
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/PoseWithCovariance.h"
#include "ros/ros.h"

struct Location {
    std::string name;
    double x;
    double y;
    double theta;
    double w;

    /**
     * @brief Construct a new Location object
     *
     * @param name The name of the location
     * @param x The x-coordinate of the location
     * @param y The y-coordinate of the location
     * @param theta The z-coordinate of the orientation of the location
     * @param w The w-coordinate of the orientation of the location
     */
    Location(
        const std::string &name, double x, double y, double theta, double w
    );

    /**
     * @brief Construct a new Location object
     *
     * @param name The name of the location
     * @param pose The pose of the location
     */
    Location(
        const std::string &name, const geometry_msgs::PoseWithCovariance &pose
    );

    /**
     * @brief Construct a new Location object
     *
     * @param location The location to copy
     */
    Location(const Location &location);

    /**
     * @brief Convert the location to a pose
     *
     * @return geometry_msgs::Pose The pose of the location
     */
    geometry_msgs::Pose to_pose() const;
};