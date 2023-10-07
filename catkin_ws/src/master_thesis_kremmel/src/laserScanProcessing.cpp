#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/transform_broadcaster.h>


#include "json.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>


using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class ICPMatcher
{
public:
    ICPMatcher(ros::NodeHandle &nTemp) : n(nTemp)
    {
        laserScanSubscriber.subscribe(n, "/velodyne_points", 10);
        curentPoseSubcriber.subscribe(n, "/odometry_node/odometry", 10);

        sync.reset(new Sync(SyncRule(10), laserScanSubscriber, currentStateSubscriber));
        sync->registerCallback(boost::bind(&ICPMatcher::searchLMmatches, this, _1, _2));

        // LM JSON Pfad und Inputstream konfigurieren
        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        jsonPath = packagePath + "/data/stored_landmarks.json";

        int NUM_LM = 100;          // Maximale Anzahl der LM's auf 100 gesetzt
        int size = 3 + 2 * NUM_LM; // Dimension der Jakobimatrizen und der Covarianzmatrizen
        // Unsicherheit der Sensorwerte aus:
        // https://dgk.badw.de/fileadmin/user_upload/Files/DGK/docs/c-826.pdf Seite 86
        Q << 0.03, 0,
            0, 0.0625;
        H_low.resize(2, 5);
        H_i.resize(2, size);
    }

    void searchLMmatches(const sensor_msgs::PointCloud2ConstPtr &laserScanMsg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &currentStateMsg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laser_scan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*laserScanMsg, *pcl_laser_scan);

        computeJacobianOfLMsInReach(currentStateMsg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
    }

    void computeJacobianOfLMsInReach(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr currentState)
    {
        std::ifstream f(jsonPath);
        stored_landmarks = json::parse(f);
        // for-Schleife durch alle gespeicherten LM's
        int count = 0;
        int i = 0;
        std::vector<int> inReach;
        double roll, pitch, yaw;
        tf::Quaternion q(
            currentState->pose.pose.orientation.x,
            currentState->pose.pose.orientation.y,
            currentState->pose.pose.orientation.z,
            currentState->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        H_i.setZero();

        for (json::iterator it = stored_landmarks["landmarks"].begin(); it != stored_landmarks["landmarks"].end(); ++it)
        {
            delta.setZero();
            delta(0) = (double)it.value()["pose"].at(0).get<float>() - currentState->pose.pose.position.x;
            delta(1) = (double)it.value()["pose"].at(1).get<float>() - currentState->pose.pose.position.y;
            std::cout << "LM:" << i << "delta_x: " << delta(0) << "delta_y: " << delta(1) << std::endl;

            if (sqrt(pow(delta(0), 2) + pow(delta(1), 2)) < 5)
            {
                count++;
                inReach.push_back(i);

                double q = delta.transpose() * delta; // Euklidische Distanz

                predictedMeasurment(0) = sqrt(q);
                predictedMeasurment(1) = atan2(delta(1), delta(0) - yaw);

                H_low << -sqrt(q) * delta(0),
                    -sqrt(q) * delta(1), 0, sqrt(q) * delta(0), sqrt(q) * delta(1),
                    delta(1), -delta(0), -q, -delta(1), -delta(0);
                H_low = H_low * 1 / q;

                H_i.block(0, 0, 2, 3) = H_low.block(0, 0, 2, 3);
                H_i(0, 3 + i) = H_low(0, 3);
                H_i(0, 4 + i) = H_low(0, 4);
                H_i(1, 3 + i) = H_low(1, 3);
                H_i(1, 4 + i) = H_low(1, 4);
            }
            i++;
        }
        std::cout << "Landmarks in reach: " << count << std::endl;
    }

private:
    ros::NodeHandle n;
    message_filters::Subscriber<sensor_msgs::PointCloud2> laserScanSubscriber;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> curentPoseSubcriber;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseWithCovarianceStamped> SyncRule;
    typedef message_filters::Synchronizer<SyncRule> Sync;
    boost::shared_ptr<Sync> sync;

    std::string jsonPath;
    json stored_landmarks;
    Eigen::Vector2d delta;
    Eigen::Vector2d predictedMeasurment;
    Eigen::Vector2d actualMeasurment;
    Eigen::MatrixXd H_low;
    Eigen::MatrixXd H_i;
    Eigen::MatrixXd K;
    Eigen::Matrix2d Q;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICPMatcher");
    ros::NodeHandle n;
    ICPMatcher Node(n);

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
/*
int main()
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    return (0);
} */