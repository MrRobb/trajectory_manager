
#include "GensimModel.hpp"
#include "gtest/gtest.h"

namespace {

TEST(GensimModelTest, load_model) {
    try {
        GensimModel model("data/gensim/model.bin");
    } catch (const std::exception &e) {
        FAIL() << "Exception thrown: " << e.what();
    }
}

TEST(GensimModelTest, load_invalid_model) {
    EXPECT_THROW(
        GensimModel model("data/gensim/invalid_model.bin"),
        std::invalid_argument
    );
}

TEST(GensimModelTest, similarity) {
    GensimModel model("data/gensim/model.bin");
    auto sim = model.similarity("cat", "dog");
    EXPECT_GT(sim, 0.0);
    EXPECT_LT(sim, 1.0);
}

TEST(GensimModelTest, similarity_invalid_word1) {
    GensimModel model("data/gensim/model.bin");
    auto sim = model.similarity("invalid_word", "dog");
    EXPECT_EQ(sim, std::numeric_limits<double>::max());
}

TEST(GensimModelTest, similarity_invalid_word2) {
    GensimModel model("data/gensim/model.bin");
    auto sim = model.similarity("cat", "invalid_word");
    EXPECT_EQ(sim, std::numeric_limits<double>::max());
}

}  // namespace