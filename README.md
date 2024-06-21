# Trajectory Manager

> WARNING: This package is still under active development and is not yet ready for use.

This package is a ROS package that provides a service to generate a trajectory for a robot to follow. The trajectory is generated in the robot's base frame and is published as a `nav_msgs/Path` message.

The trajectory is based on a set of waypoints that are provided by the user. The waypoints are provided in the robot's base frame and are used to generate a smooth trajectory that the robot can follow.

The order of the waypoints is calculated based on the semantic similarity of the object that the robot is trying to find. The waypoints are then used to generate a trajectory that the robot can follow to reach the object.


## Installation

To install the package, clone the repository into your catkin workspace and build the package using `catkin_make`.

```bash
cd ~/catkin_ws/src
git clone https://github.com/MrRobb/trajectory_manager.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release && source devel/setup.sh
```

## Usage

To use the package, you need to run the `trajectory_manager` node. The node provides a service that can be used to generate a trajectory for the robot to follow.

```bash
rosrun trajectory_manager trajectory_manager
```

## Input

The package expects the following parameters to be set in the `trajectory_manager` node:

- `node_name` (string, default: "trajectory_manager"): The name of the node.
- `locations_frame` (string, default: "map"): The frame in which the locations are defined.
- `locations_csv` (string, default: "data/locations5.csv"): The path to the CSV file that contains the locations.
- `gensim_model_file` (string, default: "data/gensim/model.bin"): The path to the Gensim model file.
- `location_update_topic` (string, default: "/amcl_pose"): The topic on which the location updates are published.
- `go_to_service` (string, default: "/robot_manager/go_to_service"): The service that the robot manager uses to go to a location.

## Output

The package provides the following services:

- `last_route_publisher_topic` (string, default: "/trajectory_manager/last_route"): The topic on which the last route is published. This service is used for debugging purposes.
- `get_current_location_service` (string, default: "/trajectory_manager/get_current_location"): The service that is used to get the current location of the robot.
- `get_route_for_object_service` (string, default: "/trajectory_manager/get_route_for_object"): The service that is used to get the route for an object.
- `go_to_location_service` (string, default: "/trajectory_manager/go_to_location"): The service that is used to go to a location.
- `add_current_location_service` (string, default: "/trajectory_manager/add_current_location"): The service that is used to add the current location to the list of locations.
- `closest_location_to_position_service` (string, default: "/trajectory_manager/closest_location_to_position"): The service that is used to get the closest location to a position.

## TODO

- [x] Manage the locations in a CSV file.
- [x] Use the AMCL pose to update the current location.
- [ ] Load the parameters dynamically from a file (rosparam or yaml).
- [ ] Use Word2Vec model to calculate the semantic similarity of the objects.