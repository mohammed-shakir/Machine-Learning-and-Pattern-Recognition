#pragma once

#include <string>
#include <vector>

#include "gesture.h"

class Model {
public:

    const std::vector<std::string> JOINT_NAMES{
        "Hip_Center",
        "Spine",
        "Shoulder_Center",
        "Head",
        "Shoulder_Left",
        "Elbow_Left",
        "Wrist_Left",
        "Hand_Left",
        "Shoulder_Right",
        "Elbow_Right",
        "Wrist_Right",
        "Hand_Right",
        "Hip_Left",
        "Knee_Left",
        "Ankle_Left",
        "Foot_Left",
        "Hip_Right",
        "Knee_Right",
        "Ankle_Right",
        "Foot_Right",
    };

    Model(std::string fileName);
    ~Model();


    void loadData(std::string fileName);
    void printData();
    void missingData();
    void visualizeData();

private:
    std::vector<Gesture> gestures{};
    std::vector<std::vector<std::string>> raw_data{};
};


