#pragma once
#include <string>
#include <vector>

class Preprocessor {
public:
    Preprocessor(std::string fileName);
    ~Preprocessor();

    void missingValues();
    void printData();
    void exportDataToCSV(const std::string& outputFileName);
    void visualizeData();

private:
    std::vector<std::vector<float>> data{};
    std::vector<std::string> gestureName{};
    std::vector<uint32_t> gestureID{};
};
