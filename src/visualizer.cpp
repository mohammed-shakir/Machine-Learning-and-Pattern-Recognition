#include "visualizer.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <iostream>
#include <thread>

void Visualizer::writeToFile(Gesture &gesture, std::string fileName)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(gesture.size());
    pcl::visualization::PCLVisualizer viewer("Gesture: " + gesture.getName());

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {

        cloud->points[i].x = gesture.getJointByIndex(i).getMeanCoors().x;
        cloud->points[i].y = gesture.getJointByIndex(i).getMeanCoors().y;
        cloud->points[i].z = gesture.getJointByIndex(i).getMeanCoors().z;
        viewer.addText3D(gesture.getJointByIndex(i).getName(), cloud->points[i], 0.01, 1.0, 0.0, 0.0, "id_" + std::to_string(i), 0);
        // Looks kinda messy, needs some more work
        for(size_t j = 0; j < gesture.getJointByIndex(i).getConnections().size(); j++)
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

    
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 100, "cloud name");
    viewer.addPointCloud(cloud, "cloud");
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}