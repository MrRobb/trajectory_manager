
#include "GensimModel.hpp"

#include "ros/console.h"

GensimModel::GensimModel(const std::string &model_file) {
    // TODO: Implement this
    ROS_WARN("GensimModel constructor not implemented");
    this->loadModel(model_file);
}

void GensimModel::loadModel(std::string model_file) {
    // TODO: Implement this
    ROS_WARN("loadModel not implemented");
}

double GensimModel::similarity(std::string word1, std::string word2) {
    // TODO: Implement this
    ROS_WARN("similarity not implemented");
    return 0.5;
}