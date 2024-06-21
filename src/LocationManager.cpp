
#include "LocationManager.hpp"

void LocationManager::load_locations() {
    // Read the locations from the CSV file
    rapidcsv::Document document(this->filename);
    for (size_t i = 0; i < document.GetRowCount(); i++) {
        std::string name = document.GetCell<std::string>("name", i);
        double x = document.GetCell<double>("x", i);
        double y = document.GetCell<double>("y", i);
        double theta = document.GetCell<double>("theta", i);
        double w = document.GetCell<double>("w", i);
        this->locations.insert_or_assign(name, Location(name, x, y, theta, w));
    }
}

void LocationManager::save_locations() {
    // Write the locations to the CSV file
    std::ofstream file(this->filename, std::ios::trunc);

    // Write the header row
    file << "name,x,y,theta,w\n";

    // Write the data rows
    for (const auto &[name, location] : this->locations) {
        file << name << "," << location.x << "," << location.y << ","
             << location.theta << "," << location.w << "\n";
    }
}

LocationManager::LocationManager(const std::string &filename) {
    // Initialize the variables
    this->locations = {};
    this->filename = filename;

    // Read the locations from the file
    this->load_locations();

    // Log the number of locations loaded
    ROS_INFO(
        "Loaded %lu locations from %s", this->locations.size(), filename.c_str()
    );
}

std::optional<Location> LocationManager::get_location(const std::string &name
) const {
    try {
        return this->locations.at(name);
    } catch (const std::out_of_range &e) {
        ROS_ERROR("Location %s not found", name.c_str());
        return std::nullopt;
    }
}

void LocationManager::add_location(const Location &location) {
    // Add the location to the locations map
    this->locations.insert_or_assign(location.name, location);

    // Save the locations to the CSV file
    this->save_locations();

    // Log the location that was added
    ROS_INFO(
        "Added location %s at (%f, %f, %f, %f) to locations CSV",
        location.name.c_str(),
        location.x,
        location.y,
        location.theta,
        location.w
    );
}

void LocationManager::remove_location(const std::string &name) {
    // Remove the location from the locations map
    this->locations.erase(name);

    // Save the locations to the CSV file
    this->save_locations();

    // Log the location that was removed
    ROS_INFO("Removed location %s from locations CSV", name.c_str());
}

void LocationManager::update_location(std::string name, Location location) {
    // Update the location in the locations map
    this->locations.insert_or_assign(name, location);

    // Save the locations to the CSV file
    this->save_locations();

    // Log the location that was updated
    ROS_INFO(
        "Updated location %s to (%f, %f, %f, %f) in locations CSV",
        name.c_str(),
        location.x,
        location.y,
        location.theta,
        location.w
    );
}

void LocationManager::clear_locations() {
    // Clear the locations map
    this->locations.clear();

    // Save the locations to the CSV file
    this->save_locations();

    // Log that the locations were cleared
    ROS_INFO("Cleared all locations from locations CSV");
}

std::vector<std::pair<double, Location>>
LocationManager::get_semantic_similarity(const std::string &object_class) {
    // TODO: Load gensim model

    /*
    Python example:
        assert os.path.exists(model_file), f"File {model_file} does not
    exist"
        rospy.loginfo(f"Loading gensim model from {model_file}")
        model = KeyedVectors.load_word2vec_format(model_file, binary=True)
        rospy.loginfo(f"Loaded gensim model with {len(model.index_to_key)}
        words")

        try:
            similarity = self.gensim_model.similarity(
                f"{location['name']}_NOUN", f"{object_class}_NOUN"
            )
            location["semantic_similarity"] = similarity
        except KeyError:
            rospy.logerr(f"Word not found in gensim model:
            {object_class}") location["semantic_similarity"] = 0.0
    */

    // TODO: For now, we just return the same similarity for all locations
    ROS_WARN("Using placeholder semantic similarity calculation");

    // Initialize the semantic similarity vector
    std::vector<std::pair<double, Location>> semantic_similarity;

    // Iterate through all locations
    for (const auto &[name, location] : this->locations) {
        // Calculate the semantic similarity between the object class and
        // the location
        double similarity = 0.5;

        // Add the semantic similarity to the vector
        semantic_similarity.push_back({similarity, location});
    }

    // Return the semantic similarity vector
    return semantic_similarity;
}

std::optional<Location> LocationManager::get_closest_location(
    const geometry_msgs::Pose &pose
) {
    // Initialize the closest location
    std::optional<Location> closest_location = std::nullopt;
    double closest_distance = std::numeric_limits<double>::max();

    // Iterate through all locations
    for (const auto &[name, location] : this->locations) {
        // Calculate the distance between the pose and the location
        double distance = sqrt(
            pow(pose.position.x - location.x, 2) +
            pow(pose.position.y - location.y, 2)
        );

        // Update the closest location if necessary
        if (distance < closest_distance) {
            closest_location = location;
            closest_distance = distance;
        }
    }

    // Return the closest location
    return closest_location;
}