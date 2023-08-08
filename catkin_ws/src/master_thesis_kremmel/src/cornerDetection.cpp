#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>

using namespace std::chrono_literals;

class LandMarkCornerDetector
{
public:
    LandMarkCornerDetector(ros::NodeHandle &nTemp) : n(nTemp)
    {
        LMsubscriber = n.subscribe<sensor_msgs::PointCloud2>("/new_landmark", 10, &LandMarkCornerDetector::newLandMarkCallback, this);
        viewer_timer = n.createTimer(ros::Duration(0.1), &LandMarkCornerDetector::timerCB, this);
        pcl::PointCloud<pcl::PointXYZ>::Ptr emptyCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        viewer = createViewer(emptyCloud_ptr);
    }

    void newLandMarkCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        ROS_INFO("++++++++++++++++++   Got new LandMark   +++++++++++++++++");
        ROS_INFO("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *pcl_cloud);
        viewer->updatePointCloud<pcl::PointXYZ>(pcl_cloud, "Landmark");;
    }

    void timerCB(const ros::TimerEvent &)
    {
        if (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
        else
        {
            ros::shutdown();
        }
    }

    pcl::visualization::PCLVisualizer::Ptr createViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        // --------------------------------------------
        // -----         Open 3D viewer           -----
        // --------------------------------------------
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Landmark Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Landmark");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        return (viewer);
    }

private:
    ros::NodeHandle n;
    ros::Timer viewer_timer;
    ros::Subscriber LMsubscriber;
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LandMarkCornerDetector");
    ros::NodeHandle n;
    LandMarkCornerDetector Node(n);
    ros::spin();
    return 0;
}