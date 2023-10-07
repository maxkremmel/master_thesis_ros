#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include "json.hpp"

using json = nlohmann::json;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class EKFSlam
{
public:
    EKFSlam(ros::NodeHandle &nTemp) : n(nTemp)
    {
        curentPoseSubcriber.subscribe(n, "/odometry_node/odometry", 10);
        laserScanSubscriber.subscribe(n, "/transformed_laserscan", 10);

        covEllipseMsgPublisher = n.advertise<visualization_msgs::Marker>("/ekf_slam/current_state_marker", 1);
        currentStatePublisher = n.advertise<nav_msgs::Odometry>("/ekf_slam/current_state", 1);
        predictionStatePublisher = n.advertise<nav_msgs::Odometry>("/ekf_slam/prediction_state", 1);

        sync.reset(new Sync(SyncRule(10), curentPoseSubcriber, laserScanSubscriber));
        sync->registerCallback(boost::bind(&EKFSlam::predict, this, _1, _2));
        listener = new tf2_ros::TransformListener(tfBuffer); // TransformListener mit Buffer initialisieren
        // PCL Viewer initialisieren
        pcl::PointCloud<pcl::PointXYZ>::Ptr emptyCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        viewer = createViewer(emptyCloud_ptr);
        // Initialisierung des EKF

        EPS = 1e-4;
        int NUM_LM = 20;          // Maximale Anzahl der LM's auf 20 gesetzt
        DIMsize = 3 + 2 * NUM_LM; // Dimension der Jakobimatrizen und der Covarianzmatrizen
        state_vector.resize(DIMsize);
        state_vector.setZero();
        state_vector_last.resize(DIMsize);
        state_vector_last.setZero();
        state_vector_prediction.setZero();

        Cov.resize(DIMsize, DIMsize);
        Cov.setZero();
        Eigen::MatrixXd diag(2 * NUM_LM, 2 * NUM_LM);
        diag.setIdentity();
        diag = diag * 10000.0;
        Cov.bottomRightCorner(2 * NUM_LM, 2 * NUM_LM) = diag;

        Q.resize(2, 2);
        Q << 0.03, 0, // Werte aus Velodyne VLP-16 Datenblatt
            0, 0.0625;

        ros::param::get("/EKFSlam/alpha1", alpha1);
        ros::param::get("/EKFSlam/alpha2", alpha2);
        ros::param::get("/EKFSlam/alpha3", alpha3);
        ros::param::get("/EKFSlam/alpha4", alpha4);

        last_timestamp = ros::Time::now().toSec();

        // LM JSON Pfad und Inputstream konfigurieren
        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        jsonPath = packagePath + "/data/stored_landmarks.json";

        ros::Duration(2).sleep(); // Warten bis rviz geladen ist
    }

    void predict(const nav_msgs::OdometryConstPtr &new_pose_msg, const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
    {
        double dt = ros::Time::now().toSec() - last_timestamp; // Zeitspanne seit letztem Funktionsaufruf berechnen
        last_timestamp = ros::Time::now().toSec();

        double theta = state_vector_last(2);

        state_vector.head(3) << new_pose_msg->pose.pose.position.x, new_pose_msg->pose.pose.position.y, getYaw(new_pose_msg); // aktuelle Pose -> berechnet von KISS-ICP

        Eigen::MatrixXd G;                                                        // Jakobi des Bewegungsmodells
        G.resize(DIMsize, DIMsize);
        G.setIdentity();
        Eigen::MatrixXd V;
        V.resize(3, 2);
        Eigen::Matrix2d M;

        // Zur berechung der Jakobi (G) aus aktueller Pose und letzter Pose Geschwindigkeiten berechnen
        Eigen::Vector3d vel3d(3);
        vel3d = (state_vector.head(3) - state_vector_last.head(3)) / dt;
        double v = sqrt(pow(vel3d(0), 2) + pow(vel3d(1), 2)); // Was passiert mit negativen Werten?
        double w = vel3d(2);

        M << alpha1 * v * v + alpha2 * w * w, 0,
            0, alpha3 * v * v + alpha4 * w * w;

        if (abs(w) > EPS) // Verhindern von Division durch 0
        {
            G.block(0, 0, 3, 3) << 1, 0, -(v / w) * cos(theta) + (v / w) * cos(theta + w * dt),
                0, 1, -(v / w) * sin(theta) + (v / w) * sin(theta + w * dt),
                0, 0, 1;
            V << (-sin(theta) + sin(theta + w * dt)) / w, (v * (sin(theta) - sin(theta + w * dt))) / w * w + (v * cos(theta + w * dt) * dt) / w,
                (cos(theta) - cos(theta + w * dt)) / w, (v * (cos(theta) - cos(theta + w * dt))) / w * w + (v * sin(theta + w * dt) * dt) / w,
                0, dt;
        }
        else
        {
            // D'Hospital Regel angewendet um numerische Fehler (Division durch Null)
            G.block(0, 0, 3, 3) << 1, 0, -v * sin(theta) * dt,
                0, 1, v * cos(theta) * dt,
                0, 0, 1;
            V << cos(theta) * dt, -v * sin(theta) * dt * dt * 0.5, // D'Hospital wurde zweimal angewendet
                sin(theta) * dt, v * cos(theta) * dt * dt * 0.5,
                0, dt;
        }

        Cov = G * Cov * G.transpose();
        Cov.block(0, 0, 3, 3) += V * M * V.transpose();

        correct(new_laserscan_msg);

        state_vector_last = state_vector; // Aktuellen Zustandsvektor in state_vector_last zwischenspeichern

        viewer->spinOnce();
        publishEllipses();
    }

    void correct(const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
    {
        std::ifstream f(jsonPath);
        json stored_landmarks = json::parse(f);

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*new_laserscan_msg, *pcl_laserscan);

        viewer->updatePointCloud(pcl_laserscan, "Laser");

        pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLandmarks(new pcl::PointCloud<pcl::PointXYZ>);
        // for-Schleife durch alle gespeicherten LM's
        int i = 0;
        int cov_index = 0;
        Eigen::MatrixXd H_i;
        H_i.resize(2, DIMsize);
        H_i.setZero();

        for (json::iterator it = stored_landmarks["landmarks"].begin(); it != stored_landmarks["landmarks"].end(); ++it)
        {
            double landmark_x = (double)it.value()["pose"].at(0).get<float>();
            double landmark_y = (double)it.value()["pose"].at(1).get<float>();
            cov_index = 3 + 2 * i;
            state_vector(cov_index) = landmark_x;
            state_vector(cov_index + 1) = landmark_y;
            Eigen::Matrix<double, 2, 1> delta = Eigen::Matrix<double, 2, 1>::Zero();
            delta(0) = landmark_x - state_vector(0);
            delta(1) = landmark_y - state_vector(1);
            // If stored LM is closer than 5 m...
            if (sqrt(pow(delta(0), 2) + pow(delta(1), 2)) < 5)
            {
                double q = delta.transpose() * delta; // Euklidische Distanz
                Eigen::Vector2d predictedMeasurment;
                predictedMeasurment(0) = sqrt(q);
                predictedMeasurment(1) = constrain_angle(atan2(delta(1), delta(0) - state_vector(3)));

                Eigen::MatrixXd H_low(2, 5);
                H_low << -sqrt(q) * delta(0), -sqrt(q) * delta(1), 0, sqrt(q) * delta(0), sqrt(q) * delta(1),
                    delta(1), -delta(0), -q, -delta(1), delta(0);
                H_low = H_low * 1 / q;

                H_i.block(0, 0, 2, 3) = H_low.block(0, 0, 2, 3);
                H_i(0, cov_index) = H_low(0, 3);
                H_i(0, cov_index + 1) = H_low(0, 4);
                H_i(1, cov_index) = H_low(1, 3);
                H_i(1, cov_index + 1) = H_low(1, 4);
                // Kalman Gain berechnen
                Eigen::MatrixXd K = Cov * H_i.transpose() * (H_i * Cov * H_i.transpose() + Q).inverse();
                // calculate actual range bearing measurement of LM throug ICP matching of laser scan and stored LM point clouds
                Eigen::Vector2d actualMeasurement = searchLMmatches(pcl_laserscan, it.value()["points"], it.value()["pose"], convergedLandmarks, cov_index);
                if (actualMeasurement(0) != -1)
                {
                    state_vector = state_vector + K * (actualMeasurement - predictedMeasurment); // state_vektor updaten
                    Cov = (MatrixXd::Identity(Cov.rows(), Cov.rows()) - K * H_i) * Cov;          // Cov updaten
                }
                else
                {
                    // skip the update step
                }
            }
            i++;
        }
        numberOfStoredLandmarks = i;
        // viewer->updatePointCloud(pcl_landmark_target, "Landmark");
        viewer->removePointCloud("Landmark");
        viewer->addPointCloud(convergedLandmarks, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Landmark");
    }

    Eigen::Vector2d searchLMmatches(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan, json points, json pose, pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLandmarks, int cov_index)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_landmark_target(new pcl::PointCloud<pcl::PointXYZ>);

        // for-Schleife durch alle Punkte
        pcl_landmark_target->points.resize(points.size());
        int i = 0;
        for (json::iterator it = points.begin(); it != points.end(); ++it)
        {
            pcl_landmark_target->points[i].x = it.value().at(0).get<float>();
            pcl_landmark_target->points[i].y = it.value().at(1).get<float>();
            pcl_landmark_target->points[i].z = it.value().at(2).get<float>();
            i++;
        }

        // ToDo: statt punktwolke zu verschieben Initial guess bei icp.align() angeben
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(1.5));
        pcl::transformPointCloud(*pcl_landmark_target, *pcl_landmark_target, tf2::transformToEigen(transform).matrix());
        // The Iterative Closest Point algorithm
        // ToDo: Fine tunen
        // ToDo: Kann die PCL den Coherent Point Drift (CPD) Algorithmus???? (https://github.com/gadomski/cpd)
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(pcl_landmark_target);
        icp.setInputTarget(pcl_laserscan);

        icp.align(*pcl_landmark_target);

        double fitnessScore = icp.getFitnessScore();
        Eigen::Vector2d actualMeasurement;
        if (fitnessScore < 0.01)
        {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            Eigen::Affine3f affineTransform;
            affineTransform.matrix() = transformation;

            pcl::PointXYZ landmarkPose(pose.at(0).get<float>(), pose.at(1).get<float>(), pose.at(2).get<float>());
            landmarkPose = pcl::transformPoint(landmarkPose, affineTransform);
            // Update the LM poses in the state vector
            state_vector(cov_index) = landmarkPose.x;
            state_vector(cov_index + 1) = landmarkPose.y;

            Eigen::Vector2d delta_actual;
            delta_actual(0) = landmarkPose.x - state_vector(0);
            delta_actual(1) = landmarkPose.y - state_vector(1);

            double q = delta_actual.transpose() * delta_actual;

            actualMeasurement(0) = sqrt(q);
            actualMeasurement(1) = constrain_angle(atan2(delta_actual(1), delta_actual(0) - state_vector(3)));

            *convergedLandmarks += *pcl_landmark_target;
        }
        else
        {
            actualMeasurement(0) = -1;
        }
        return actualMeasurement;
    }

    double constrain_angle(double radian)
    {
        if (radian < -M_PI)
        {
            radian += 2 * M_PI;
        }
        else if (radian > M_PI)
        {
            radian -= 2 * M_PI;
        }
        return radian;
    }

    double getYaw(const nav_msgs::OdometryConstPtr odomMsg)
    {
        double roll, pitch, yaw;
        tf2::Quaternion q(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }

    void publishEllipses()
    {
        double poseMajor, poseMinor, poseTheta;
        poseEllipse(poseMajor, poseMinor, poseTheta);
        publishBelieve(state_vector(0), state_vector(1), poseTheta, poseMinor, poseMajor, "pose estimate");

        double x, y, lmMajor, lmMinor, lmTheta;
        for (int i = 0; i < numberOfStoredLandmarks; i++)
        {
            landmarkEllipse(i, x, y, lmMinor, lmMajor, lmTheta);
            if (lmMinor < 100 || lmMajor < 100)
            {
                publishBelieve(x, y, lmTheta, lmMinor, lmMajor, "lm estimates", i);
            }
        }
    }

    void ellipse(Eigen::MatrixXd X, double &major, double &minor, double &theta)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> a(X);
        double e0 = sqrt(a.eigenvalues()(0));
        double e1 = sqrt(a.eigenvalues()(1));
        if (e0 > e1)
        {
            theta = atan2(a.eigenvectors()(1, 0), a.eigenvectors()(0, 0));
            major = e0;
            minor = e1;
        }
        else
        {
            theta = atan2(a.eigenvectors()(1, 1), a.eigenvectors()(0, 1));
            major = e1;
            minor = e0;
        }
    }

    void poseEllipse(double &major, double &minor, double &theta)
    {
        ellipse(Cov.block(0, 0, 2, 2), major, minor, theta);
    }

    void landmarkEllipse(int id, double &x, double &y, double &minor, double &major, double &theta)
    {
        int idx = 3 + id * 2;
        x = state_vector(idx);
        y = state_vector(idx + 1);
        ellipse(Cov.block(idx, idx, 2, 2), major, minor, theta);
    }

    void publishBelieve(double x, double y, double theta, double minor, double major, std::string ellipseName, int id = 0)
    {
        //  state_vektor und Cov in Odometry Message verpacken und publishen
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta);
        geometry_msgs::Pose ellipsePose;
        ellipsePose.position.x = x;
        ellipsePose.position.y = y;
        ellipsePose.position.z = 0;
        ellipsePose.orientation = tf2::toMsg(quat);

        if (ellipseName == "pose estimate")
        {
            nav_msgs::Odometry currentStateMsg;
            currentStateMsg.pose.pose = ellipsePose;
            currentStateMsg.child_frame_id = "base_link";
            currentStateMsg.header.frame_id = "odom";
            currentStateMsg.header.stamp = ros::Time::now();

            currentStatePublisher.publish(currentStateMsg);
        }

        // rviz Marker generieren
        visualization_msgs::Marker covEllipseMsg;
        covEllipseMsg.header.frame_id = "odom";
        covEllipseMsg.header.stamp = ros::Time::now();
        // Marker mit selbem Namespace und id überschreiben sich selber (das wollen wir)
        covEllipseMsg.ns = ellipseName;
        covEllipseMsg.id = id;
        covEllipseMsg.type = visualization_msgs::Marker::SPHERE;
        covEllipseMsg.action = visualization_msgs::Marker::ADD;
        covEllipseMsg.pose = ellipsePose;
        covEllipseMsg.scale.x = minor * 0.5;
        covEllipseMsg.scale.y = major * 0.5;
        covEllipseMsg.scale.z = 0.01;
        covEllipseMsg.color.r = 1.0f;
        covEllipseMsg.color.g = 0.0f;
        covEllipseMsg.color.b = 1.0f;
        covEllipseMsg.color.a = 0.5;
        covEllipseMsg.lifetime = ros::Duration();

        covEllipseMsgPublisher.publish(covEllipseMsg);
    }

    pcl::visualization::PCLVisualizer::Ptr createViewer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
    {
        // -----         Open 3D viewer           -----
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Landmark Viewer"));
        viewer->setBackgroundColor(0.05, 0.05, 0.05);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        // Add laser point cloud dummy and ncrease point size and set color of Laser Cloud
        viewer->addPointCloud(cloud, "Laser");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Laser");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "Laser");
        viewer->setCameraPosition(5.7, 12, 10.6, -0.27, -0.47, 0.84, 0);
        viewer->setSize(1920, 1080); // Visualiser window size
        return (viewer);
    }

private:
    ros::NodeHandle n;
    ros::Publisher covEllipseMsgPublisher;
    ros::Publisher currentStatePublisher;
    ros::Publisher predictionStatePublisher;

    message_filters::Subscriber<sensor_msgs::PointCloud2> laserScanSubscriber;
    message_filters::Subscriber<nav_msgs::Odometry> curentPoseSubcriber;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> SyncRule;
    typedef message_filters::Synchronizer<SyncRule> Sync;
    boost::shared_ptr<Sync> sync;

    double last_timestamp; // Zeitspanne die zwischen den Funktionsaufrufen vergeht
    int DIMsize;
    int EPS; // Threshhold under Omega is treated as zero.
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    Eigen::Vector3d state_vector_prediction;
    Eigen::VectorXd state_vector;      // Zustands Vektor der Pose und der Landmarken
    Eigen::VectorXd state_vector_last; // Zustands Vektor der Pose und der Landmarken bei letztem Aufruf der Predict Methode
    Eigen::MatrixXd Cov;               // Covarianzmatrix des Zustandsvekotors (Roboterpose und Landmarken)
    int numberOfStoredLandmarks;

    std::string jsonPath;

    Eigen::Matrix2d Q;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    ros::Timer viewer_timer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *listener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    EKFSlam Node(n);

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}