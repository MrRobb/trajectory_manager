
#include <string>

class GensimModel {
private:
    /**
     * @brief Load the Word2Vec model from the given file
     *
     * @param model_file The path to the Word2Vec model binary file
     */
    void loadModel(std::string model_file);

public:
    /**
     * @brief Construct a new Gensim Model object
     *
     * @param model_file The path to the Word2Vec model binary file
     */
    GensimModel(const std::string &model_file);

    /**
     * @brief Calculate the similarity between two words
     *
     * @param word1 The first word
     * @param word2 The second word
     * @return double The similarity between the two words
     */
    double similarity(std::string word1, std::string word2);
};