#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include "json.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class NewLandMarkProcessing
{
public:
    NewLandMarkProcessing(ros::NodeHandle &nTemp) : n(nTemp)
    {
        LMsubscriber = n.subscribe<sensor_msgs::PointCloud2>("/new_landmark", 10, &NewLandMarkProcessing::newLandMarkCallback, this);
        LMpublisher = n.advertise<sensor_msgs::PointCloud2>("/stored_landmarks", 1000);
        /* viewer_timer = n.createTimer(ros::Duration(0.1), &NewLandMarkProcessing::timerCB, this);
        pcl::PointCloud<pcl::PointXYZ>::Ptr emptyCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        viewer = createViewer(emptyCloud_ptr); */

        // LM's aus JSON Datei einlesen
        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        jsonPath = packagePath + "/data/stored_landmarks.json";
        std::ifstream f(jsonPath);
        stored_landmarks = json::parse(f);

        // Deklerieren der PointCloud2 Message zum Visualisieren der gespeicherten LM's
        sensor_msgs::PointCloud2Modifier pc2_modifier(landmark); // Modifier to describe what the fields are.
        pc2_modifier.setPointCloud2Fields(3,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32);
        // PointCloud2 Message deklarieren
        landmark.header = std_msgs::Header();
        landmark.header.frame_id = "odom";
        landmark.height = 1;
        landmark.point_step = 12;
        landmark.is_bigendian = true;
        landmark.is_dense = false;

        ros::Duration(2).sleep(); // Warten bis rviz geladen ist

        publishLandMarks(stored_landmarks); // Alle bereits gespeicherten LM's in rviz visualisieren
    }

    void publishLandMarks(json landmarks)
    {
        json points; // json Objekt für alle Punkte aller LM's
        // for-Schleife durch alle gespeicherten LM's
        for (json::iterator it = stored_landmarks["landmarks"].begin(); it != stored_landmarks["landmarks"].end(); ++it)
        {
            // for-Schleife durch die Punkte einer LM
            for (json::iterator point_it = it.value()["points"].begin(); point_it != it.value()["points"].end(); ++point_it)
            {
                points.push_back(point_it.value());
            }
        }
        // Nachflogend wird die PointCloud2 Messeage mit allen Punkten befüllt und gepublished
        landmark.width = static_cast<u_int>(points.size());
        landmark.row_step = landmark.width * landmark.point_step;
        landmark.data.clear();
        landmark.data.resize(landmark.row_step);
        // Iterators for PointCloud msg
        sensor_msgs::PointCloud2Iterator<float> iterX(landmark, "x");
        sensor_msgs::PointCloud2Iterator<float> iterY(landmark, "y");
        sensor_msgs::PointCloud2Iterator<float> iterZ(landmark, "z");
        json::iterator it = points.begin();
        // iterate over the message and populate the fields.
        for (; iterX != iterX.end(); ++iterX, ++iterY, ++iterZ, ++it)
        {
            *iterX = it.value().at(0);
            *iterY = it.value().at(1);
            *iterZ = it.value().at(2);
        }
        landmark.header.stamp = ros::Time::now();
        LMpublisher.publish(landmark);
    }

    void newLandMarkCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        ROS_INFO("++++++++++++++++++   Got new LandMark   +++++++++++++++++");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *pcl_cloud);
        /* viewer->updatePointCloud<pcl::PointXYZ>(pcl_cloud, "Landmark"); */

        // Downsample of LM Pointcloud:
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(pcl_cloud);
        sor.setLeafSize(0.1f, 0.1f, 0.1f);
        sor.filter(*pcl_cloud);

        // Store LM:
        // Calculate Pose of LM using mean of XYZ coordinates off all poits
        float x_mean = 0;
        float y_mean = 0;
        float z_mean = 0;
        json lm_pointcloud;

        for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cloud->begin(); it != pcl_cloud->end(); it++)
        {
            if (it->z < 0.1)
            {
                // Skip this point because its part of the floor
            }
            else
            {
                x_mean += it->x;
                y_mean += it->y;
                z_mean += it->z;

                json point = {it->x, it->y, it->z};
                lm_pointcloud.push_back(point);
            }

            // ToDo: Eventuell noch Punktwolke komprimieren
        }
        if(lm_pointcloud.empty()){
            std::cout << "New Landmark only consists out of floor... not storing LM" << std::endl;
            return;
        }
        
        x_mean /= lm_pointcloud.size();
        y_mean /= lm_pointcloud.size();
        z_mean /= lm_pointcloud.size();

        std::cout << "Looking if LM is allready initialized" << std::endl;
        // iterate through LM's
        bool store_new_landmark = true;
        for (json::iterator it = stored_landmarks["landmarks"].begin(); it != stored_landmarks["landmarks"].end(); ++it)
        {
            // Wenn sich neue LM näher als 20 cm in jede Richtung bei bestehender LM befindet nicht speichern
            if (abs(it.value()["pose"].at(0).get<float>() - x_mean) < 0.2 &&
                abs(it.value()["pose"].at(1).get<float>() - y_mean) < 0.2 &&
                abs(it.value()["pose"].at(2).get<float>() - z_mean) < 0.2)
            {
                // ToDo: Evetuell noch prüfen ob Punktwolken übereinstimmen und nur wenn nicht auch speichern
                std::cout << "LM already stored" << std::endl;
                store_new_landmark = false;
                break;
            }
        }

        if (store_new_landmark)
        {
            std::cout << "LM is new, therefore storing new LM in JSON" << std::endl;
            json new_landmark = {
                {"pose", {x_mean, y_mean, z_mean}},
                {"points", lm_pointcloud}};

            stored_landmarks["landmarks"].push_back(new_landmark);
            storeNewLandmark();
            publishLandMarks(stored_landmarks);
        }
    }

    void storeNewLandmark()
    {
        // Save PointCloud in JSON
        std::ofstream stored_landmarks_stream(jsonPath, std::ios::out | std::ios::trunc);
        if (!stored_landmarks_stream.is_open())
        {
            std::cerr << "*** error: could not open output file\n";
        }
        else
        {
            stored_landmarks_stream << std::setw(4) << stored_landmarks << std::endl;
            stored_landmarks_stream.flush();
            stored_landmarks_stream.close();
            std::cout << "++++   new LM written to JSON successfully!    ++++" << std::endl;
        }
    }

    /* void timerCB(const ros::TimerEvent &)
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
        // -----         Open 3D viewer           -----
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Landmark Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Landmark");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        return (viewer);
    } */

private:
    ros::NodeHandle n;
    /* ros::Timer viewer_timer; */
    ros::Subscriber LMsubscriber;
    ros::Publisher LMpublisher;
    /* pcl::visualization::PCLVisualizer::Ptr viewer; */
    std::string jsonPath;
    json stored_landmarks;
    sensor_msgs::PointCloud2 landmark;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "NewLandMarkProcessing");
    ros::NodeHandle n;
    NewLandMarkProcessing Node(n);
    ros::spin();
    return 0;
}