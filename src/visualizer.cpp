#include "visualizer.h"
#include <string>
#include <iostream>
#include <thread>

void Visualizer::visualizeGesture(Gesture &gesture, pcl::visualization::PCLVisualizer viewer, pcl::PointCloud<pcl::PointXYZ> cloud)
{

    for (auto i = 0; i < cloud->points.size(); ++i)
    {

        cloud->points[i].x = gesture.getJointByIndex(i).getMeanCoors().x;
        cloud->points[i].y = gesture.getJointByIndex(i).getMeanCoors().y;
        cloud->points[i].z = gesture.getJointByIndex(i).getMeanCoors().z;
        viewer.addText3D(gesture.getJointByIndex(i).getName(), cloud->points[i], 0.01, 1.0, 0.0, 0.0, "id_" + std::to_string(i), 0);
        
        // Looks kinda messy, needs some more work
        for(auto j = 0; j < gesture.getJointByIndex(i).getConnections().size(); j++)
        {
            pcl::PointXYZ p1, p2;
            p1.x = gesture.getJointByIndex(i).getMeanCoors().x;
            p1.y = gesture.getJointByIndex(i).getMeanCoors().y;
            p1.z = gesture.getJointByIndex(i).getMeanCoors().z;
            p2.x = gesture.getJointByIndex(i).getConnections()[j]->getMeanCoors().x;
            p2.y = gesture.getJointByIndex(i).getConnections()[j]->getMeanCoors().y;
            p2.z = gesture.getJointByIndex(i).getConnections()[j]->getMeanCoors().z;
            viewer.addLine(p1, p2, "id_" + std::to_string(i) + "_" + std::to_string(j), 0);
        }
    }
}

void Visualizer::visualizeGestureByName(std::string gestureName, std::vector<Gesture> &gestures)
{
    pcl::PointCloud<pcl::PointXYZ> cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 5;
    cloud->height = 1;
    // Fill in the cloud data
    pcl::visualization::PCLVisualizer viewer(new pcl::visualization::PCLVisualizer("Gesture: " + gestureName));
    for(auto i = 0; i < gestures.size(); i++)
    {
        if(gestures[i].getName() == gestureName)
        {
            cloud->points.resize(cloud->points.size() + gesture.size());
            visualizeGesture(gestures[i], viewer, cloud);
        }
    }

    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}