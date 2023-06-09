#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include <set>
#include "model.h"

#define ITEM_SIZE 60

Model::Model(std::string fileName) {
    loadData(fileName);
    createConnections();
}

Model::~Model() {}

void Model::loadData(std::string fileName)
{
    std::ifstream file(fileName);

    if (!file.is_open()) {
        std::runtime_error("Could not open file");
    }

    std::string row;

    while (std::getline(file, row)) {
        raw_data.push_back(std::vector<std::string>());
        std::stringstream ss(row);
        std::string col;

        while (std::getline(ss, col, ','))
        {
            raw_data.back().push_back(col);
        }
    }

    file.close();

    for (auto row_i = 0; row_i < raw_data.size(); row_i++)
    {
        Gesture gesture = Gesture();
        std::string name = raw_data[row_i][raw_data[row_i].size() - 2];
        std::transform(name.begin(), name.end(), name.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });
        gesture.setName(name);
        gesture.setId(std::stoi(raw_data[row_i][raw_data[row_i].size() - 1]));

        Averages meanAverages = Averages();
        Averages stdAverages = Averages();
        for (auto col_i = 0; col_i < ITEM_SIZE; col_i += 3)
        {
            Coordinate meanCoors;
            Coordinate stdCoors;

            meanCoors.x = raw_data[row_i][col_i] != "" ? std::stof(raw_data[row_i][col_i]) : std::numeric_limits<float>::infinity();
            meanCoors.y = raw_data[row_i][col_i + 1] != "" ? std::stof(raw_data[row_i][col_i + 1]) : std::numeric_limits<float>::infinity();
            meanCoors.z = raw_data[row_i][col_i + 2] != "" ? std::stof(raw_data[row_i][col_i + 2]) : std::numeric_limits<float>::infinity();

            meanAverages.x += meanCoors.x != std::numeric_limits<float>::infinity() ? meanCoors.x : 0;
            meanAverages.y += meanCoors.y != std::numeric_limits<float>::infinity() ? meanCoors.y : 0;
            meanAverages.z += meanCoors.z != std::numeric_limits<float>::infinity() ? meanCoors.z : 0;

            stdCoors.x = raw_data[row_i][col_i + ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE]) : std::numeric_limits<float>::infinity();
            stdCoors.y = raw_data[row_i][col_i + ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE + 1]) : std::numeric_limits<float>::infinity();
            stdCoors.z = raw_data[row_i][col_i + ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE + 2]) : std::numeric_limits<float>::infinity();

            stdAverages.x += stdCoors.x != std::numeric_limits<float>::infinity() ? stdCoors.x : 0;
            stdAverages.y += stdCoors.y != std::numeric_limits<float>::infinity() ? stdCoors.y : 0;
            stdAverages.z += stdCoors.z != std::numeric_limits<float>::infinity() ? stdCoors.z : 0;

            meanCoors.x_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE]) : std::numeric_limits<float>::infinity();
            meanCoors.y_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE + 1]) : std::numeric_limits<float>::infinity();
            meanCoors.z_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE + 2]) : std::numeric_limits<float>::infinity();

            meanAverages.x_angle += meanCoors.x_angle != std::numeric_limits<float>::infinity() ? meanCoors.x_angle : 0;
            meanAverages.y_angle += meanCoors.y_angle != std::numeric_limits<float>::infinity() ? meanCoors.y_angle : 0;
            meanAverages.z_angle += meanCoors.z_angle != std::numeric_limits<float>::infinity() ? meanCoors.z_angle : 0;

            stdCoors.x_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE]) : std::numeric_limits<float>::infinity();
            stdCoors.y_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE + 1]) : std::numeric_limits<float>::infinity();
            stdCoors.z_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE + 2]) : std::numeric_limits<float>::infinity();

            stdAverages.x_angle += stdCoors.x_angle != std::numeric_limits<float>::infinity() ? stdCoors.x_angle : 0;
            stdAverages.y_angle += stdCoors.y_angle != std::numeric_limits<float>::infinity() ? stdCoors.y_angle : 0;
            stdAverages.z_angle += stdCoors.z_angle != std::numeric_limits<float>::infinity() ? stdCoors.z_angle : 0;

            meanCoors.type = Coordinate::Type::MEAN;
            stdCoors.type = Coordinate::Type::STD;

            // std::cout << "Processing joint: " << JOINT_NAMES[col_i / 3] << " (row: " << row_i << ", col: " << col_i << ")" << std::endl;
            gesture.addJoint(meanCoors, stdCoors, JOINT_NAMES[col_i / 3]);
        }

        meanAverages.x /= ITEM_SIZE / 3;
        meanAverages.y /= ITEM_SIZE / 3;
        meanAverages.z /= ITEM_SIZE / 3;
        meanAverages.x_angle /= ITEM_SIZE / 3;
        meanAverages.y_angle /= ITEM_SIZE / 3;
        meanAverages.z_angle /= ITEM_SIZE / 3;

        stdAverages.x /= ITEM_SIZE / 3;
        stdAverages.y /= ITEM_SIZE / 3;
        stdAverages.z /= ITEM_SIZE / 3;
        stdAverages.x_angle /= ITEM_SIZE / 3;
        stdAverages.y_angle /= ITEM_SIZE / 3;
        stdAverages.z_angle /= ITEM_SIZE / 3;

        // std::cout << "Amount of joints: " << gesture.size() << std::endl;

        gestures.push_back(gesture);
    }
}

void Model::missingData()
{
    // std::cout << "Gestures size: " << gestures.size() << std::endl;
    // std::cout << "Gestures size: " << gestures.size() << std::endl;
    // std::cout << "Gesture 0 name: " << gestures[0].getName() << std::endl;
    // std::cout << "Gesture 0 joint count: " << gestures[0].size() << std::endl;
    for (auto i = 0; i < gestures.size(); i++)
    {
        // std::cout << "Joint length: " << gestures[i].size() << std::endl;
        for (auto j = 0; j < gestures[i].size(); j++)
        {
            Joint &joint = gestures[i].getJointByIndex(j);
            Coordinate &joint_mean_coors = joint.getMeanCoors();
            Coordinate &joint_std_coors = joint.getStdCoors();

            if (joint_mean_coors.x == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.x = gestures[i].getMeanAverages().x;
            }
            if (joint_mean_coors.y == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.y = gestures[i].getMeanAverages().y;
            }

            if (joint_mean_coors.z == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.z = gestures[i].getMeanAverages().z;
            }

            if (joint_mean_coors.x_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.x_angle = gestures[i].getMeanAverages().x_angle;
            }

            if (joint_mean_coors.y_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.y_angle = gestures[i].getMeanAverages().y_angle;
            }

            if (joint_mean_coors.z_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.z_angle = gestures[i].getMeanAverages().z_angle;
            }

            if (joint_std_coors.x == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.x = gestures[i].getStdAverages().x;
            }

            if (joint_std_coors.y == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.y = gestures[i].getStdAverages().y;
            }

            if (joint_std_coors.z == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.z = gestures[i].getStdAverages().z;
            }

            if (joint_std_coors.x_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.x_angle = gestures[i].getStdAverages().x_angle;
            }

            if (joint_std_coors.y_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.y_angle = gestures[i].getStdAverages().y_angle;
            }

            if (joint_std_coors.z_angle == std::numeric_limits<float>::infinity())
            {
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.z_angle = gestures[i].getStdAverages().z_angle;
            }

            joint.setMeanCoors(joint_mean_coors);
            joint.setStdCoors(joint_std_coors);
        }
    }
}

void Model::createConnections()
{
    for (auto i = 0; i < gestures.size(); i++)
    {
        for (auto j = 0; j < gestures[i].size(); j++)
        {
            auto &joint = gestures[i].getJointByIndex(j);
            auto connections = JOINT_CONNECTIONS.find(joint.getName());
            if (connections != JOINT_CONNECTIONS.end())
            {
                for (auto &connectionName : connections->second)
                {
                    for (auto k = 0; k < gestures[i].size(); k++)
                    {
                        auto &connection = gestures[i].getJointByIndex(k);
                        if (connection.getName() == connectionName)
                        {
                            joint.addConnection(&connection);
                            break;
                        }
                    }
                }
            }
        }
    }
}

void Model::removeSimilarGestures() {
    std::cout << "Current gestures size: " << gestures.size() << std::endl;
    size_t oldSize = gestures.size();
    for (size_t i = 0; i < gestures.size() - 1; i++) {
        if(i == gestures.size() - 1) break;
        for (size_t j = i + 1; j < gestures.size(); j++) {
            if (isGestureSimilar(gestures[i], gestures[j])) {
                std::cout << "Removing gesture " << gestures[j].getName() << std::endl;
                gestures.erase(gestures.begin() + j);
                j--;
            }
        }
    }

    std::cout << "New gestures size: " << gestures.size() << std::endl;
    std::cout << "Removed " << oldSize - gestures.size() << " gestures" << std::endl;
}

bool Model::isGestureSimilar(Gesture gesture1, Gesture gesture2){
    float gesture1Values = 0;
    float gesture2Values = 0;

    for(size_t i = 0; i < gesture1.size(); i++){
        gesture1Values += static_cast<float>(gesture1.getJointByIndex(i).getMeanCoors().x + gesture1.getJointByIndex(i).getMeanCoors().y + gesture1.getJointByIndex(i).getMeanCoors().z + gesture1.getJointByIndex(i).getMeanCoors().x_angle + gesture1.getJointByIndex(i).getMeanCoors().y_angle + gesture1.getJointByIndex(i).getMeanCoors().z_angle);
        gesture2Values += static_cast<float>(gesture2.getJointByIndex(i).getMeanCoors().x + gesture2.getJointByIndex(i).getMeanCoors().y + gesture2.getJointByIndex(i).getMeanCoors().z + gesture2.getJointByIndex(i).getMeanCoors().x_angle + gesture2.getJointByIndex(i).getMeanCoors().y_angle + gesture2.getJointByIndex(i).getMeanCoors().z_angle);
    }

    if(gesture1Values > gesture2Values){
        return gesture1Values - gesture2Values < 0.01;
    } else {
        return gesture2Values - gesture1Values < 0.01;
    }
}

void Model::listAllGestures()
{
    std::set<std::string> uniqueGestures;
    for (auto i = 0; i < gestures.size(); i++)
    {
        uniqueGestures.insert(gestures[i].getName());
    }

    std::string result = "";
    for (const auto& gestureName : uniqueGestures)
    {
        result += gestureName + ", ";
    }

    // Remove the trailing comma and space
    if (!result.empty())
    {
        result.pop_back();
        result.pop_back();
    }

    std::cout << result << std::endl;
}

void Model::printData()
{
    std::cout << "Starting print " << std::endl;
    for (auto i = 0; i < gestures.size(); i++)
    {
        std::cout << "Gesture: " << gestures[i].getName() << std::endl;
        for (auto j = 0; j < gestures[i].size(); j++)
        {
            std::cout << "Joint: " << gestures[i].getJointByIndex(j).getName() << std::endl;
            std::cout << "Mean Coors: " << gestures[i].getJointByIndex(j).getMeanCoors().x << " " << gestures[i].getJointByIndex(j).getMeanCoors().y << " " << gestures[i].getJointByIndex(j).getMeanCoors().z << " " << gestures[i].getJointByIndex(j).getMeanCoors().x_angle << " " << gestures[i].getJointByIndex(j).getMeanCoors().y_angle << " " << gestures[i].getJointByIndex(j).getMeanCoors().z_angle << std::endl;
            std::cout << "Std Coors: " << gestures[i].getJointByIndex(j).getStdCoors().x << " " << gestures[i].getJointByIndex(j).getStdCoors().y << " " << gestures[i].getJointByIndex(j).getStdCoors().z << " " << gestures[i].getJointByIndex(j).getStdCoors().x_angle << " " << gestures[i].getJointByIndex(j).getStdCoors().y_angle << " " << gestures[i].getJointByIndex(j).getStdCoors().z_angle << std::endl;
        }
    }
}

void Model::exportData(std::string fileName) {
    std::cout << "Exporting data to " << fileName << std::endl;

    // Export data to csv file
    std::ofstream file;
    file.open(fileName);

    // Fill the file with data
    for (auto i = 0; i < gestures.size(); i++)
    {
        if(i == 0){
            file << "gesture";
            for (auto j = 0; j < gestures[i].size(); j++)
            {
                file << "," << "mean_x_" << gestures[i].getJointByIndex(j).getName() << "," "mean_y_" << gestures[i].getJointByIndex(j).getName() << "," << "mean_z_" << gestures[i].getJointByIndex(j).getName() << "," << "mean_x_angle_" << gestures[i].getJointByIndex(j).getName() << "," << "mean_y_angle_" << gestures[i].getJointByIndex(j).getName() << "," << "mean_z_angle_" << gestures[i].getJointByIndex(j).getName() << "," << "std_x_" << gestures[i].getJointByIndex(j).getName() << "," << "std_y_" << gestures[i].getJointByIndex(j).getName() << "," << "std_z_" << gestures[i].getJointByIndex(j).getName() << "," << "std_x_angle_" << gestures[i].getJointByIndex(j).getName() << "," << "std_y_angle_" << gestures[i].getJointByIndex(j).getName() << "," << "std_z_angle_" << gestures[i].getJointByIndex(j).getName();
            }
            file << "\n";
        }
        file << gestures[i].getName();
        for (auto j = 0; j < gestures[i].size(); j++)
        {
             file << "," << gestures[i].getJointByIndex(j).getMeanCoors().x << "," << gestures[i].getJointByIndex(j).getMeanCoors().y << "," << gestures[i].getJointByIndex(j).getMeanCoors().z << "," << gestures[i].getJointByIndex(j).getMeanCoors().x_angle << "," << gestures[i].getJointByIndex(j).getMeanCoors().y_angle << "," << gestures[i].getJointByIndex(j).getMeanCoors().z_angle << "," << gestures[i].getJointByIndex(j).getStdCoors().x << "," << gestures[i].getJointByIndex(j).getStdCoors().y << "," << gestures[i].getJointByIndex(j).getStdCoors().z << "," << gestures[i].getJointByIndex(j).getStdCoors().x_angle << "," << gestures[i].getJointByIndex(j).getStdCoors().y_angle << "," << gestures[i].getJointByIndex(j).getStdCoors().z_angle;
        }
        file << "\n";
    }

    file.close();

    std::cout << "Data exported to " << fileName << std::endl;
}

void Model::visualizeGestureByName(std::string gestureName){
    visualizer.visualizeGestureByName(gestureName, gestures);
}
