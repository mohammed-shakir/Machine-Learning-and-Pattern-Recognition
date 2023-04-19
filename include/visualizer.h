#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <memory>

#include "gesture.h"

class Visualizer{
public:
    void visualizeGesture(Gesture &gesture, pcl::visualization::PCLVisualizer &viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, size_t cloudIndex);
    void visualizeGestureByName(std::string gestureName, std::vector<Gesture> &gestures);
};