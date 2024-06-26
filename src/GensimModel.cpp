
#include "GensimModel.hpp"

#include <filesystem>

#include "ros/console.h"

GensimModel::GensimModel(const std::string& model_file) {
    // Check if model file exists
    if (!std::filesystem::exists(model_file)) {
        ROS_ERROR("Model file does not exist: %s", model_file.c_str());
        throw std::invalid_argument("Model file does not exist");
    }

    // Load model
    this->model.load(model_file);
    ROS_INFO("Loaded Gensim model from %s", model_file.c_str());
    ROS_INFO("Model contains %lu words", this->model.map().size());
}

double GensimModel::similarity(
    const std::string& word1, const std::string& word2,
    const std::string& word1_type, const std::string& word2_type
) {
    // Map word 1 to vector 1 (if word 1 is not in the model, return infinity)
    auto w1vec = this->model.map().find(word1 + "_NOUN");
    if (w1vec == this->model.map().end()) {
        ROS_ERROR("Word %s not found in model", word1.c_str());
        return std::numeric_limits<double>::max();
    }

    // Map word 2 to vector 2 (if word 2 is not in the model, return infinity)
    auto w2vec = this->model.map().find(word2 + "_NOUN");
    if (w2vec == this->model.map().end()) {
        ROS_ERROR("Word %s not found in model", word2.c_str());
        return std::numeric_limits<double>::max();
    }

    // Calculate the similarity between the two vectors
    auto distance = this->model.distance(w1vec->second, w2vec->second);
    return distance;
}