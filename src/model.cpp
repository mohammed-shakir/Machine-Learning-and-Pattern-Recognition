#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>
#include "model.h"

#define ITEM_SIZE 60

Model::Model(std::string fileName)
{
    loadData(fileName);
}

Model::~Model()
{
}

void Model::loadData(std::string fileName){
    std::ifstream file(fileName);

    if (!file.is_open())
    {
        std::runtime_error("Could not open file");
    }
    std::string row;
    while (std::getline(file, row))
    {
        raw_data.push_back(std::vector<std::string>());
        std::stringstream ss(row);
        std::string col;

        while(std::getline(ss, col, ','))
        {
            raw_data.back().push_back(col);
        }
        
    }

    file.close();

    for(size_t row_i = 0; row_i < raw_data.size(); row_i++)
    {
        Gesture gesture = Gesture();
        std::string name = raw_data[row_i][raw_data[row_i].size() - 2];
        std::transform(name.begin(), name.end(), name.begin(),
        [](unsigned char c){ return std::tolower(c); });
        gesture.setName(name);
        gesture.setId(std::stoi(raw_data[row_i][raw_data[row_i].size() - 1]));

        Averages meanAverages;
        Averages stdAverages;
        for(size_t col_i = 0; col_i < ITEM_SIZE; col_i += 3)
        {
            Coordinate meanCoors;
            Coordinate stdCoors;
            
            meanCoors.x = raw_data[row_i][col_i] != "" ? std::stof(raw_data[row_i][col_i]) : std::numeric_limits<float>::quiet_NaN();
            meanCoors.y = raw_data[row_i][col_i + 1] != "" ? std::stof(raw_data[row_i][col_i + 1]) : std::numeric_limits<float>::quiet_NaN();
            meanCoors.z = raw_data[row_i][col_i + 2] != "" ? std::stof(raw_data[row_i][col_i + 2]) : std::numeric_limits<float>::quiet_NaN();

            meanAverages.x += meanCoors.x != std::numeric_limits<float>::quiet_NaN() ? meanCoors.x : 0;
            meanAverages.y += meanCoors.y != std::numeric_limits<float>::quiet_NaN() ? meanCoors.y : 0;
            meanAverages.z += meanCoors.z != std::numeric_limits<float>::quiet_NaN() ? meanCoors.z : 0;

            stdCoors.x = raw_data[row_i][col_i + ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE]) : std::numeric_limits<float>::quiet_NaN();
            stdCoors.y = raw_data[row_i][col_i + ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE + 1]) : std::numeric_limits<float>::quiet_NaN();
            stdCoors.z = raw_data[row_i][col_i + ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + ITEM_SIZE + 2]) : std::numeric_limits<float>::quiet_NaN();

            stdAverages.x += stdCoors.x != std::numeric_limits<float>::quiet_NaN() ? stdCoors.x : 0;
            stdAverages.y += stdCoors.y != std::numeric_limits<float>::quiet_NaN() ? stdCoors.y : 0;
            stdAverages.z += stdCoors.z != std::numeric_limits<float>::quiet_NaN() ? stdCoors.z : 0;

            meanCoors.x_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE]) : std::numeric_limits<float>::quiet_NaN();
            meanCoors.y_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE + 1]) : std::numeric_limits<float>::quiet_NaN();
            meanCoors.z_angle = raw_data[row_i][col_i + 2 * ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + 2 * ITEM_SIZE + 2]) : std::numeric_limits<float>::quiet_NaN();

            meanAverages.x_angle += meanCoors.x_angle != std::numeric_limits<float>::quiet_NaN() ? meanCoors.x_angle : 0;
            meanAverages.y_angle += meanCoors.y_angle != std::numeric_limits<float>::quiet_NaN() ? meanCoors.y_angle : 0;
            meanAverages.z_angle += meanCoors.z_angle != std::numeric_limits<float>::quiet_NaN() ? meanCoors.z_angle : 0;

            stdCoors.x_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE]) : std::numeric_limits<float>::quiet_NaN();
            stdCoors.y_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE + 1] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE + 1]) : std::numeric_limits<float>::quiet_NaN();
            stdCoors.z_angle = raw_data[row_i][col_i + 3 * ITEM_SIZE + 2] != "" ? std::stof(raw_data[row_i][col_i + 3 * ITEM_SIZE + 2]) : std::numeric_limits<float>::quiet_NaN();

            stdAverages.x_angle += stdCoors.x_angle != std::numeric_limits<float>::quiet_NaN() ? stdCoors.x_angle : 0;
            stdAverages.y_angle += stdCoors.y_angle != std::numeric_limits<float>::quiet_NaN() ? stdCoors.y_angle : 0;
            stdAverages.z_angle += stdCoors.z_angle != std::numeric_limits<float>::quiet_NaN() ? stdCoors.z_angle : 0;

            meanCoors.type = Coordinate::Type::MEAN;
            stdCoors.type = Coordinate::Type::STD;

            gesture.addJoint(meanCoors, stdCoors, JOINT_NAMES[col_i/3]);
        }

        meanAverages.x /= ITEM_SIZE/3;
        meanAverages.y /= ITEM_SIZE/3;
        meanAverages.z /= ITEM_SIZE/3;
        meanAverages.x_angle /= ITEM_SIZE/3;
        meanAverages.y_angle /= ITEM_SIZE/3;
        meanAverages.z_angle /= ITEM_SIZE/3;

        stdAverages.x /= ITEM_SIZE/3;
        stdAverages.y /= ITEM_SIZE/3;
        stdAverages.z /= ITEM_SIZE/3;
        stdAverages.x_angle /= ITEM_SIZE/3;
        stdAverages.y_angle /= ITEM_SIZE/3;
        stdAverages.z_angle /= ITEM_SIZE/3;

        gestures.push_back(gesture);
    }
}

void Model::missingData(){
    for (size_t i = 0; i < gestures.size(); i++)
    {
        for(int j = 0; j < ITEM_SIZE; j += 3){
            Joint joint = gestures[i].getJointByIndex(j);

            Coordinate joint_mean_coors = joint.getMeanCoors();
            Coordinate joint_std_coors = joint.getStdCoors();


            if(joint_mean_coors.x == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.x = gestures[i].getMeanAverages().x;
            }
            if(joint_mean_coors.y == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.y = gestures[i].getMeanAverages().y;
            }

            if(joint_mean_coors.z == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.z = gestures[i].getMeanAverages().z;
            }

            if(joint_mean_coors.x_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.x_angle = gestures[i].getMeanAverages().x_angle;
            }

            if(joint_mean_coors.y_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.y_angle = gestures[i].getMeanAverages().y_angle;
            }

            if(joint_mean_coors.z_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_mean_coors.z_angle = gestures[i].getMeanAverages().z_angle;
            }

            if(joint_std_coors.x == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.x = gestures[i].getStdAverages().x;
            }

            if(joint_std_coors.y == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.y = gestures[i].getStdAverages().y;
            }


            if(joint_std_coors.z == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.z = gestures[i].getStdAverages().z;
            }

            if(joint_std_coors.x_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.x_angle = gestures[i].getStdAverages().x_angle;
            }

            if(joint_std_coors.y_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.y_angle = gestures[i].getStdAverages().y_angle;
            }

            if(joint_std_coors.z_angle == std::numeric_limits<float>::quiet_NaN()){
                std::cout << "Missing data in gesture " << gestures[i].getName() << " joint " << gestures[i].getJointByIndex(j).getName() << std::endl;
                joint_std_coors.z_angle = gestures[i].getStdAverages().z_angle;
            }

            gestures[i].getJointByIndex(j).setMeanCoors(joint_mean_coors);
            gestures[i].getJointByIndex(j).setStdCoors(joint_std_coors);
        }
    }
}

void Model::printData()
{
    for (size_t i = 0; i < gestures.size(); i++)
    {
        std::cout << "Gesture: " << gestures[i].getName() << std::endl;
        for(int j = 0; j < ITEM_SIZE; j += 3){
            std::cout << "Joint: " << gestures[i].getJointByIndex(j).getName() << std::endl;
            std::cout << "Mean Coors: " << gestures[i].getJointByIndex(j).getMeanCoors().x << " " << gestures[i].getJointByIndex(j).getMeanCoors().y << " " << gestures[i].getJointByIndex(j).getMeanCoors().z << " " << gestures[i].getJointByIndex(j).getMeanCoors().x_angle << " " << gestures[i].getJointByIndex(j).getMeanCoors().y_angle << " " << gestures[i].getJointByIndex(j).getMeanCoors().z_angle << std::endl;
            std::cout << "Std Coors: " << gestures[i].getJointByIndex(j).getStdCoors().x << " " << gestures[i].getJointByIndex(j).getStdCoors().y << " " << gestures[i].getJointByIndex(j).getStdCoors().z << " " << gestures[i].getJointByIndex(j).getStdCoors().x_angle << " " << gestures[i].getJointByIndex(j).getStdCoors().y_angle << " " << gestures[i].getJointByIndex(j).getStdCoors().z_angle << std::endl;

        }
    }
}

void Model::visualizeData()
{
}