#include "visualizer.h"
#include <string>
#include <iostream>
#include <thread>

void Visualizer::visualizeGesture(Gesture &gesture, pcl::visualization::PCLVisualizer &viewer, const std::string &cloudId, bool isFirstGesture)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = gesture.size();
    cloud->height = 1;
    cloud->points.resize(gesture.size());

    for (auto jointIndex = 0; jointIndex < gesture.size(); jointIndex++)
    {
        cloud->points[jointIndex].x = gesture.getJointByIndex(jointIndex).getMeanCoors().x;
        cloud->points[jointIndex].y = gesture.getJointByIndex(jointIndex).getMeanCoors().y;
        cloud->points[jointIndex].z = gesture.getJointByIndex(jointIndex).getMeanCoors().z;
        if (isFirstGesture)
        {
            viewer.addText3D(gesture.getJointByIndex(jointIndex).getName(), cloud->points[jointIndex], 0.01, 1.0, 0.0, 0.0, "id_" + cloudId + "_" + std::to_string(jointIndex), 0);
        }

        for (auto j = 0; j < gesture.getJointByIndex(jointIndex).getConnections().size(); j++)
        {
            pcl::PointXYZ p1, p2;
            p1.x = gesture.getJointByIndex(jointIndex).getMeanCoors().x;
            p1.y = gesture.getJointByIndex(jointIndex).getMeanCoors().y;
            p1.z = gesture.getJointByIndex(jointIndex).getMeanCoors().z;
            p2.x = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().x;
            p2.y = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().y;
            p2.z = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().z;
            viewer.addLine(p1, p2, "id_" + cloudId + "_" + std::to_string(jointIndex) + "_" + std::to_string(j));
        }
    }

    viewer.addPointCloud(cloud, cloudId);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloudId);
}

void Visualizer::visualizeGestureByName(std::string gestureName, std::vector<Gesture> &gestures)
{
    pcl::visualization::PCLVisualizer viewer("Gesture: " + gestureName);

    int gestureCounter = 0;
    for (auto &gesture : gestures)
    {
        if (gesture.getName() == gestureName)
        {
            std::string cloudId = "cloud_" + std::to_string(gestureCounter);
            visualizeGesture(gesture, viewer, cloudId, gestureCounter == 0);
            gestureCounter++;
        }
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
