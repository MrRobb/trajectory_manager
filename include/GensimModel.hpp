
#pragma once

#include <string>

#include "word2vec.hpp"

class GensimModel {
private:
    /**
     * @brief The Word2Vec model
     */
    w2v::w2vModel_t model;

public:
    /**
     * @brief Construct a new Gensim Model object
     *
     * @param model_file The path to the Word2Vec model binary file
     */
    GensimModel(const std::string& model_file);

    /**
     * @brief Calculate the similarity between two words
     *
     * @param word1 The first word
     * @param word2 The second word
     * @return double The similarity between the two words
     */
    double similarity(
        const std::string& word1, const std::string& word2,
        const std::string& word1_type = "NOUN",
        const std::string& word2_type = "NOUN"
    );
};