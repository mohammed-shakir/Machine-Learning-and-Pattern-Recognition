#include "visualizer.h"
#include <string>
#include <iostream>
#include <thread>

void Visualizer::visualizeGesture(Gesture &gesture, pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t cloudIndex)
{

    for (auto i = cloudIndex; i < (cloudIndex + 20); i++)
    {
        for(auto jointIndex = 0; jointIndex < gesture.size(); jointIndex++){
            cloud->points[i].x = gesture.getJointByIndex(jointIndex).getMeanCoors().x;
            cloud->points[i].y = gesture.getJointByIndex(jointIndex).getMeanCoors().y;
            cloud->points[i].z = gesture.getJointByIndex(jointIndex).getMeanCoors().z;
            viewer.addText3D(gesture.getJointByIndex(jointIndex).getName(), cloud->points[i], 0.01, 1.0, 0.0, 0.0, "id_" + std::to_string(i)+"_"+std::to_string(jointIndex), 0);
            
            // Looks kinda messy, needs some more work
            for(auto j = 0; j < gesture.getJointByIndex(jointIndex).getConnections().size(); j++)
            {
                pcl::PointXYZ p1, p2;
                p1.x = gesture.getJointByIndex(jointIndex).getMeanCoors().x;
                p1.y = gesture.getJointByIndex(jointIndex).getMeanCoors().y;
                p1.z = gesture.getJointByIndex(jointIndex).getMeanCoors().z;
                p2.x = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().x;
                p2.y = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().y;
                p2.z = gesture.getJointByIndex(jointIndex).getConnections()[j]->getMeanCoors().z;
                viewer.addLine(p1, p2, "id_" + std::to_string(i)+"_"+std::to_string(jointIndex) + "_" + std::to_string(j), 0);
            }
        }
    }
}

void Visualizer::visualizeGestureByName(std::string gestureName, std::vector<Gesture> &gestures)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 5;
    cloud->height = 1;
    // Fill in the cloud data
    cloud->points.resize(gestures[0].size());
    pcl::visualization::PCLVisualizer viewer("Gesture: " + gestureName);
    bool isFirst = true;
    for(auto i = 0; i < gestures.size(); i ++)
    {
        if(gestures[i].getName() == gestureName)
        {
            if(!isFirst){
                cloud->points.resize(cloud->points.size() + gestures[i].size());
            }
            isFirst = false;
            visualizeGesture(gestures[i], viewer, cloud, cloud->points.size() - gestures[i].size());
        }
    }

    viewer.addPointCloud(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    while (!viewer.wasStopped())
    {
        // viewer.spinOnce(100);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}