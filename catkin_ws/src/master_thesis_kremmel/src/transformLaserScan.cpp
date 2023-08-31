#include "ros/ros.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

class TransformLaserScan
{
public:
    TransformLaserScan(ros::NodeHandle &nTemp) : n(nTemp)
    {
        // Publisher und Subscriber initialisieren
        LaserScanSub = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &TransformLaserScan::callback, this);
        LaserScanPub = n.advertise<sensor_msgs::PointCloud2>("/transformed_laserscan", 1000);
        listener = new tf2_ros::TransformListener(tfBuffer); // TransformListener mit Buffer initialisieren

        try
        {
            transform = tfBuffer.lookupTransform("base_link", "velodyne", ros::Time(0), ros::Duration(3.0)); // Nach Transformation suchen (Timeout von 3 Sekunden)
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    // Die callback Funktion transformiert jede Message des LiDAR Scanners vom velodyne Frame in den base_link Frame
    void callback(const sensor_msgs::PointCloud2ConstPtr &input_scan)
    {
        pcl::PointCloud<pcl::PointXYZ> pcl_input_scan;
        pcl::fromROSMsg(*input_scan, pcl_input_scan);       // Message des Laserscanners von einem ROS Datentypen in einen PCL Datentypen kovertieren
        pcl::PointCloud<pcl::PointXYZ> pcl_transfromed_scan;
        pcl::transformPointCloud(pcl_input_scan, pcl_transfromed_scan, tf2::transformToEigen(transform).matrix()); // Transformation erfolgt hier
        sensor_msgs::PointCloud2 transfromed_scan;          
        pcl::toROSMsg(pcl_transfromed_scan, transfromed_scan); // Transformierte Punktwolke in einen ROS Datentyp konvertieren
        LaserScanPub.publish(transfromed_scan);     // Publishen des transformierten Laserscans
    }

private:
    ros::NodeHandle n;
    ros::Subscriber LaserScanSub;
    ros::Publisher LaserScanPub;
    geometry_msgs::TransformStamped transform; // Transformation von velodyne Frame zu base_link Frame
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *listener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "TransformLaserScan");
    ros::NodeHandle n;
    TransformLaserScan Node(n);
    ros::spin();
    return 0;
}