#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKFSlam
{
public:
    EKFSlam(ros::NodeHandle &nTemp) : n(nTemp)
    {
        curentPoseSubcriber = n.subscribe<geometry_msgs::PoseStamped>("/odometry_node/odometry", 10, &EKFSlam::predict, this);

        ros::Duration(2).sleep(); // Warten bis rviz geladen ist
    }

    void predict(geometry_msgs::PoseStamped current_pose)
    {
        
        // Example usage of eigen library
        MatrixXd m = MatrixXd::Random(3, 3);
        m = (m + MatrixXd::Constant(3, 3, 1.2)) * 50;
        std::cout << "m =" << std::endl
                  << m << std::endl;
        VectorXd v(3);
        v << 1, 2, 3;
        std::cout << "m * v =" << std::endl
                  << m * v << std::endl;

    }

private:
    ros::Subscriber curentPoseSubcriber;
    ros::NodeHandle n;
    VectorXd state_vector;      // Statevektor -> Pose des Roboters und der @Landmarken sortiert nach index@(@->noch nicht endgültig)
    MatrixXd covariance_matrix; // Kovarianzmatrix der Pose und der Landmarken
    geometry_msgs::PoseStamped current_pose;

    // currentPose (ersetzt Fahrbefehl in der EKF Prediction)
    // currentLaserScan
    // predicedMeasurement (Landmarks in base_link)
};

ros::Rate loop_rate(10);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    EKFSlam Node(n);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}