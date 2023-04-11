#pragma once
#include <string>
#include <vector>

class Preprocessor {
public:
    Preprocessor(std::string fileName);
    ~Preprocessor();

    void missingValues();
    void printData();

private:
    std::vector<std::vector<float>> data{};
    std::vector<std::string> gestureName{};
    std::vector<uint32_t> gestureID{};
};
