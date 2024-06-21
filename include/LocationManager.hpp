
#include <fstream>
#include <optional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "Location.hpp"
#include "rapidcsv.h"
#include "ros/console.h"

class LocationManager {
private:
    std::string filename;
    std::unordered_map<std::string, Location> locations;

    /**
     * @brief Load the locations from the CSV file
     */
    void load_locations();

    /**
     * @brief Save the locations to the CSV file. This function overwrites the CSV.
     */
    void save_locations();

public:
    /**
     * @brief Construct a new Location Manager object
     *
     * @param filename The filename of the CSV file to load the locations from
     */
    LocationManager(const std::string &filename);

    /**
     * @brief Get the location with the given name
     *
     * @param name The name of the location
     * @return std::optional<Location> The location with the given name
     */
    std::optional<Location> get_location(const std::string &name) const;

    /**
     * @brief Add a location to the location manager. This function updates the
     * CSV file.
     *
     * @param location The location to add
     */
    void add_location(const Location &location);

    /**
     * @brief Remove a location from the location manager. If the location does
     * not exist, do nothing. This function updates the CSV file.
     *
     * @param name The name of the location to remove
     */
    void remove_location(const std::string &name);

    /**
     * @brief Update a location in the location manager. This function updates
     * the CSV file.
     *
     * @param name The name of the location to update
     * @param location The updated location
     */
    void update_location(std::string name, Location location);

    /**
     * @brief Remove all locations from the location manager. This function
     * updates the CSV file.
     */
    void clear_locations();

    /**
     * @brief Get the semantic similarity between the given object class and the
     * locations
     *
     * @param object_class The object class to compare to
     * @return std::vector<std::pair<double, Location>> A vector of the semantic
     * similarity between each location and the object class and the location
     */
    std::vector<std::pair<double, Location>> get_semantic_similarity(
        const std::string &object_class
    );

    /**
     * @brief Get the closest location to the given pose
     *
     * @param pose The pose to compare to
     * @return std::optional<Location> The closest location to the pose
     */
    std::optional<Location> get_closest_location(const geometry_msgs::Pose &pose
    );
};