#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKFSlam
{
public:
    EKFSlam(ros::NodeHandle &nTemp) : n(nTemp)
    {
        imuSubscriber = n.subscribe<sensor_msgs::Imu>("/taurob_tracker/imu/data", 50, &EKFSlam::predict, this);
        cmdVelSubscriber = n.subscribe<geometry_msgs::Twist>("/taurob_tracker/cmd_vel_raw", 10, &EKFSlam::update_cmd_vel, this);

        EPS = 1e-4;
        int NUM_LM = 20;          // Maximale Anzahl der LM's auf 20 gesetzt
        DIMsize = 3 + 2 * NUM_LM; // Dimension der Jakobimatrizen und der Covarianzmatrizen
        state_vector.resize(DIMsize);
        state_vector.setZero();
        state_vector_vel.setZero();
        state_vector_vel_last.setZero();
        state_vector_quat_last = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());

        Cov.resize(DIMsize, DIMsize);
        Cov.setZero();
        Eigen::MatrixXd diag(2 * NUM_LM, 2 * NUM_LM);
        diag.setIdentity();
        diag = diag * 10000.0;
        Cov.bottomRightCorner(2 * NUM_LM, 2 * NUM_LM) = diag;

        Q.resize(2, 2);
        Q << 0.03, 0, // Werte aus Velodyne VLP-16 Datenblatt
            0, 0.0625;

        firstRun = true;

        ros::param::get("/EKFSlam/alpha1", alpha1);
        ros::param::get("/EKFSlam/alpha2", alpha2);
        ros::param::get("/EKFSlam/alpha3", alpha3);
        ros::param::get("/EKFSlam/alpha4", alpha4);

        ros::Duration(2).sleep(); // Warten bis rviz geladen ist
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
    }

    void update_cmd_vel(const geometry_msgs::TwistConstPtr &new_cmd_vel_msg)
    {
        if (new_cmd_vel_msg->linear.x == 0 && new_cmd_vel_msg->angular.z == 0)
        {
            state_vector_vel_last.setZero();
        }
    }

    void predict(const sensor_msgs::ImuConstPtr &new_imu_msg)
    {
        last_seq = new_imu_msg->header.seq;
        if (firstRun)
        {
            last_timestamp = new_imu_msg->header.stamp.toSec();
            firstRun = false;
            return;
        }
        double dt = new_imu_msg->header.stamp.toSec() - last_timestamp; // Zeitspanne seit letztem Funktionsaufruf berechnen
        last_timestamp = new_imu_msg->header.stamp.toSec();

        Eigen::Matrix3d R;

        double qw = state_vector_quat_last.w();
        double qx = state_vector_quat_last.x();
        double qy = state_vector_quat_last.y();
        double qz = state_vector_quat_last.z();

        Eigen::Vector3d a(new_imu_msg->linear_acceleration.x, new_imu_msg->linear_acceleration.y, 0);
        Eigen::Vector3d w(new_imu_msg->angular_velocity.x, new_imu_msg->angular_velocity.y, new_imu_msg->angular_velocity.z);

        Eigen::Quaterniond quat;
        quat = Eigen::AngleAxisd(w(0) * dt, Eigen::Vector3d::UnitX()) *
               Eigen::AngleAxisd(w(1) * dt, Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(w(2) * dt, Eigen::Vector3d::UnitZ());

        R << pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy),
            2 * (qx * qy + qw * qz), pow(qw, 2) - pow(qx, 2) + pow(qy, 2) - pow(qz, 2), 2 * (qy * qz - qw * qx),
            2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2);

        state_vector_pos = state_vector_pos_last + state_vector_vel_last * dt + (R * a) * pow(dt, 2) * 0.5;
        state_vector_vel = state_vector_vel_last + (R * a) * dt;
        quat = state_vector_quat_last * quat;

        state_vector_pos_last = state_vector_pos;
        state_vector_vel_last = state_vector_vel;
        state_vector_quat_last = quat;

        state_vector(0) = state_vector_pos(0);
        state_vector(1) = state_vector_pos(1);
        state_vector(2) = getYaw(quat);
    }

    double getYaw(const nav_msgs::OdometryConstPtr odomMsg)
    {
        double tmp, yaw;
        tf2::Quaternion q(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(tmp, tmp, yaw);

        return yaw;
    }
    double getYaw(Eigen::Quaterniond quat)
    {
        double tmp, yaw;
        tf2::Quaternion q(
            quat.x(),
            quat.y(),
            quat.z(),
            quat.w());
        tf2::Matrix3x3 m(q);
        m.getRPY(tmp, tmp, yaw);

        return yaw;
    }
    geometry_msgs::TransformStamped getTransformStamped()
    {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = state_vector_pos(0);
        transformStamped.transform.translation.y = state_vector_pos(1);
        transformStamped.transform.translation.z = 0;
        transformStamped.transform.rotation = tf2::toMsg(state_vector_quat_last);
        return transformStamped;
    }

private:
    ros::NodeHandle n;

    ros::Subscriber imuSubscriber;
    ros::Subscriber cmdVelSubscriber;

    double last_timestamp; // Zeitspanne die zwischen den Funktionsaufrufen vergeht
    int DIMsize;
    int EPS; // Threshhold under Omega is treated as zero.
    int last_seq;
    bool firstRun;
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    Eigen::Vector2d cmd_vel;
    Eigen::Vector3d state_vector_pos;
    Eigen::Vector3d state_vector_pos_last;
    Eigen::Vector3d state_vector_vel;
    Eigen::Vector3d state_vector_vel_last;
    Eigen::Quaterniond state_vector_quat_last;
    Eigen::VectorXd state_vector; // Zustands Vektor der Pose und der Landmarken
    Eigen::MatrixXd Cov;          // Covarianzmatrix des Zustandsvekotors (Roboterpose und Landmarken)
    int numberOfStoredLandmarks;

    Eigen::Matrix2d Q;

    geometry_msgs::TransformStamped transformStamped;
    nav_msgs::Odometry currentStateMsg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    EKFSlam Node(n);

    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();

        transformStamped = Node.getTransformStamped();
        br.sendTransform(transformStamped);

        r.sleep();
    }
    return 0;
}