#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include "gesture.h"
#include "visualizer.h"

class Model
{
public:
    const std::vector<std::string> JOINT_NAMES{
        "Head",
        "Shoulder_Center",
        "Shoulder_Left",
        "Shoulder_Right",
        "Elbow_Left",
        "Elbow_Right",
        "Wrist_Left",
        "Wrist_Right",
        "Hand_Left",
        "Hand_Right",
        "Spine",
        "Hip_Center",
        "Hip_Left",
        "Hip_Right",
        "Knee_Left",
        "Knee_Right",
        "Ankle_Left",
        "Ankle_Right",
        "Foot_Left",
        "Foot_Right",
    };

    const std::unordered_map<std::string, std::vector<std::string>> JOINT_CONNECTIONS{
        {"Head", {"Shoulder_Center"}},
        {"Shoulder_Center", {"Shoulder_Left", "Shoulder_Right", "Spine", "Head"}},
        {"Shoulder_Left", {"Elbow_Left", "Shoulder_Center"}},
        {"Shoulder_Right", {"Elbow_Right", "Shoulder_Center"}},
        {"Elbow_Left", {"Wrist_Left", "Shoulder_Left"}},
        {"Elbow_Right", {"Wrist_Right", "Shoulder_Right"}},
        {"Wrist_Left", {"Hand_Left", "Elbow_Left"}},
        {"Wrist_Right", {"Hand_Right", "Elbow_Right"}},
        {"Hand_Left", {"Wrist_Left"}},
        {"Hand_Right", {"Wrist_Right"}},
        {"Spine", {"Hip_Center", "Shoulder_Center"}},
        {"Hip_Center", {"Hip_Left", "Hip_Right", "Spine"}},
        {"Hip_Left", {"Knee_Left", "Hip_Center"}},
        {"Hip_Right", {"Knee_Right", "Hip_Center"}},
        {"Knee_Left", {"Ankle_Left", "Hip_Left"}},
        {"Knee_Right", {"Ankle_Right", "Hip_Right"}},
        {"Ankle_Left", {"Foot_Left", "Knee_Left"}},
        {"Ankle_Right", {"Foot_Right", "Knee_Right"}},
        {"Foot_Left", {"Ankle_Left"}},
        {"Foot_Right", {"Ankle_Right"}},
    };

    Model(std::string fileName);
    ~Model();

    void loadData(std::string fileName);
    void createConnections();
    void printData();
    void missingData();
    void visualizeGestureByName(std::string gestureName);
    void removeSimilarGestures();
    bool isGestureSimilar(Gesture gesture1, Gesture gesture2);
    void listAllGestures();
private:
    std::vector<Gesture> gestures{};
    std::vector<std::vector<std::string>> raw_data{};

    Visualizer visualizer = Visualizer();
};
