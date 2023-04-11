#include "preprocessor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>

#define LINE_SIZE 240

Preprocessor::Preprocessor(std::string fileName) {
    std::ifstream file(fileName);

    if (!file.is_open()) {
        std::runtime_error("Could not open file");
    }

    std::string line;
    int lineIndex = 0;
    while (std::getline(file, line)) {
        
        // Use a stringstream to process the line
        std::istringstream lineStream(line);

        // Read each cell separated by a comma
        std::string cell;
        float Inf = std::numeric_limits<float>::infinity();
        int cellIndex = 0;
        float line_average = 0;
        data.push_back(std::vector<float>());
        while (std::getline(lineStream, cell, ',')) {
            if(cellIndex == LINE_SIZE){
                gestureName.push_back(cell);
            } else if(cellIndex == LINE_SIZE + 1){
                gestureID.push_back(std::stoi(cell));
            } else {
                try{
                    data[lineIndex].push_back(std::stof(cell));
                    line_average += data[lineIndex][data[lineIndex].size()-1];
                } catch (std::invalid_argument) {
                    data[lineIndex].push_back(Inf);
                }
            }
            cellIndex++;
        }
        data[lineIndex].push_back(line_average / LINE_SIZE);
        lineIndex++;
    }

    file.close();
}

Preprocessor::~Preprocessor() {
}

void Preprocessor::printData() {
    std::cout << "Data size: " << std::endl;
    std::cout << gestureID.size() << std::endl;
    for (uint32_t i = 0; i < data.size(); i++) {
        std::cout << "Data for gesture " << gestureName[i] << " with ID " << gestureID[i] << ": " << std::endl;
        for(uint32_t j = 0; j < data[i].size(); j++){
            std::cout << data[i][j];
        }
        std::cout << std::endl;
    }
}

void Preprocessor::missingValues() {
    float Inf = std::numeric_limits<float>::infinity();

    for(size_t i = 0; i < data.size(); i++){
        for(size_t j = 0; j < data[i].size(); j++){
            if(data[i][j] == Inf){
                data[i][j] = data[i][data[i].size() - 1];
                std::cout << "Missing value found at line " << i << " and column " << j << std::endl;
                std::cout << "Replacing with average value: " << data[i][j] << std::endl;
            }
        }
    }
}

