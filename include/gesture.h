#pragma once

#include <string>
#include <vector>

#include "coordinate.h"

struct Averages{
    float x;
    float y;
    float z;

    float x_angle;
    float y_angle;
    float z_angle;
};

class Joint
{
public:
    Joint(Coordinate meanCoors, Coordinate stdCoors, std::string name) : meanCoors(meanCoors), stdCoors(stdCoors), name(name) {};

    Coordinate getMeanCoors() const { return meanCoors; }
    Coordinate getStdCoors() const { return stdCoors; }

    void setMeanCoors(Coordinate coors) { meanCoors = coors; }
    void setStdCoors(Coordinate coors) { stdCoors = coors; }

    std::string getName() const { return name; }

private:
    std::string name;
    Coordinate meanCoors;
    Coordinate stdCoors;
};

class Gesture
{
public:

    void addJoint(Coordinate meanCoors, Coordinate stdCoors, std::string name)
    {
        joints.push_back(Joint(meanCoors, stdCoors, name));
        numJoints++;
    }

    void setName(std::string name)
    {
        this->name = name;
    }

    void setId(uint32_t id)
    {
        this->id = id;
    }

    std::string getName() const { return name; }
    uint32_t getId() const { return id; }

    Joint getJointByIndex(int index) const { return joints[index]; }

    Averages getMeanAverages() const { return meanAverages; }
    Averages getStdAverages() const { return stdAverages; }

    void setMeanAverages(Averages averages) { meanAverages = averages; }
    void setStdAverages(Averages averages) { stdAverages = averages; }

    size_t size() const { return numJoints; }

private:
    std::vector<Joint> joints{};
    size_t numJoints = 0;
    std::string name;
    uint32_t id;

    Averages meanAverages;
    Averages stdAverages;
};