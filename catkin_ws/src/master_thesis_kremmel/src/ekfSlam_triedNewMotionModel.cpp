#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
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
        cmdVelSubscriber = n.subscribe<geometry_msgs::Twist>("/taurob_tracker/cmd_vel_raw", 10, &EKFSlam::update_u, this);
        laserScanSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/transformed_laserscan", 10, &EKFSlam::laserScanCallback, this);

        covEllipseMsgPublisher = n.advertise<visualization_msgs::Marker>("/ekf_slam/current_state_marker", 1);

        listener = new tf2_ros::TransformListener(tfBuffer); // TransformListener mit Buffer initialisieren
        // PCL Viewer initialisieren
        pcl::PointCloud<pcl::PointXYZ>::Ptr emptyCloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        viewer = createViewer(emptyCloud_ptr);

        pcl_laserscan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

        // Initialisierung des EKF
        int NUM_LM = 20;          // Maximale Anzahl der LM's auf 20 gesetzt
        DIMsize = 3 + 2 * NUM_LM; // Dimension der Jakobimatrizen und der Covarianzmatrizen

        state_vector.resize(DIMsize);
        state_vector.setZero();
        state_vector_last.resize(DIMsize);
        state_vector_last.setZero();

        Cov.resize(DIMsize, DIMsize);
        Cov.setZero();
        Eigen::MatrixXd diag(2 * NUM_LM, 2 * NUM_LM);
        diag.setIdentity();
        diag = diag * 1000.0;
        Cov.bottomRightCorner(2 * NUM_LM, 2 * NUM_LM) = diag;

        alpha1 = 0.1;
        alpha2 = 0.1;
        alpha3 = 0.1;
        alpha4 = 0.1;

        Q.resize(2, 2);
        Q << 0.03, 0,
            0, 0.0625;

        H_low.resize(2, 5);
        H_low.setZero();
        H_i.resize(2, DIMsize);
        H_i.setZero();

        I.resize(DIMsize, DIMsize);
        I = MatrixXd::Identity(DIMsize, DIMsize);

        last_timestamp = ros::Time::now().toSec();

        /* std::cout << "Cov: " << std::endl
                  << Cov << std::endl
                  << std::endl;
        std::cout << "G: " << std::endl
                  << G << std::endl
                  << std::endl;
        std::cout << "R: " << std::endl
                  << R << std::endl
                  << std::endl;
        std::cout << "state_vector: " << std::endl
                  << state_vector << std::endl
                  << std::endl;
        std::cout << "H_low: " << std::endl
                  << H_low << std::endl
                  << std::endl;
        std::cout << "H_i: " << std::endl
                  << H_i << std::endl
                  << std::endl;
        std::cout << "Q: " << std::endl
                  << Q << std::endl
                  << std::endl; */

        // LM JSON Pfad und Inputstream konfigurieren
        std::string packagePath = ros::package::getPath("master_thesis_kremmel");
        jsonPath = packagePath + "/data/stored_landmarks.json";

        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "base_link";
    }

    void laserScanCallback(const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
    {
        pcl::fromROSMsg(*new_laserscan_msg, *pcl_laserscan);
    }

    void update_u(const geometry_msgs::TwistConstPtr &new_cmd_vel_msg)
    {
        v = new_cmd_vel_msg->linear.x;
        w = new_cmd_vel_msg->angular.z * 0.41;
    }

    void predict()
    {
        double dt = ros::Time::now().toSec() - last_timestamp; // Zeitspanne seit letztem Funktionsaufruf berechnen
        last_timestamp = ros::Time::now().toSec();
        double theta = state_vector_last(2);

        Eigen::Vector3d mu_last = state_vector_last.head(3);
        Eigen::Vector3d mu;

        Eigen::Vector3d new_movement;
        Eigen::MatrixXd G; // Jakobi des Bewegungsmodells
        G.resize(DIMsize, DIMsize);
        G.setIdentity();
        Eigen::MatrixXd V;
        V.resize(3, 2);
        Eigen::Matrix2d M;

        if (w == 0)
        {
            new_movement(0) = cos(theta) * v * dt;
            new_movement(1) = sin(theta) * v * dt;
            new_movement(2) = w * dt;

            V << 0, 0,
                0, 0,
                0, dt;

            M << alpha1 * v * v + alpha2 * w * w, 0,
                0, alpha3 * v * v + alpha4 * w * w;
        }
        else
        {
            new_movement(0) = -v / w * sin(theta) + v / w * sin(theta + w * dt);
            new_movement(1) = +v / w * cos(theta) - v / w * cos(theta + w * dt);
            new_movement(2) = w * dt;

            G.block(0, 0, 3, 3) << 1, 0, -(v / w) * cos(theta) + (v / w) * cos(theta + w * dt),
                0, 1, -(v / w) * sin(theta) + (v / w) * sin(theta + w * dt),
                0, 0, 1;

            V << (-sin(theta) + sin(theta + w * dt)) / w, (v * (sin(theta) - sin(theta + w * dt))) / w * w + (v * cos(theta + w * dt) * dt) / w,
                (cos(theta) - cos(theta + w * dt)) / w, (v * (cos(theta) - cos(theta + w * dt))) / w * w + (v * sin(theta + w * dt) * dt) / w,
                0, dt;

            M << alpha1 * v * v + alpha2 * w * w, 0,
                0, alpha3 * v * v + alpha4 * w * w;
        }
        Cov = G * Cov * G.transpose();
        Cov.block(0, 0, 3, 3) += V * M * V.transpose();

        state_vector.head(3) = mu_last + new_movement;
        state_vector_last = state_vector; // Aktuellen Zustandsvektor in state_vector_last zwischenspeichern

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.transform.translation.x = state_vector(0);
        transformStamped.transform.translation.y = state_vector(1);
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, state_vector(2));
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);

        correct();
        viewer->spinOnce();
        publishBelieve();
    }

    void correct()
    {
        std::ifstream f(jsonPath);
        stored_landmarks = json::parse(f);

        viewer->updatePointCloud(pcl_laserscan, "Laser");

        pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLandmarks(new pcl::PointCloud<pcl::PointXYZ>);
        // for-Schleife durch alle gespeicherten LM's
        int i = 0;
        H_i.setZero();

        for (json::iterator it = stored_landmarks["landmarks"].begin(); it != stored_landmarks["landmarks"].end(); ++it)
        {
            delta.setZero();
            delta(0) = (double)it.value()["pose"].at(0).get<float>() - state_vector(0);
            delta(1) = (double)it.value()["pose"].at(1).get<float>() - state_vector(1);

            if (sqrt(pow(delta(0), 2) + pow(delta(1), 2)) < 5)
            {
                std::cout << "Landmark: " << i << " in reach ";

                double q = delta.transpose() * delta; // Euklidische Distanz

                predictedMeasurment(0) = sqrt(q);
                predictedMeasurment(1) = atan2(delta(1), delta(0) - state_vector(3));

                H_low << -sqrt(q) * delta(0),
                    -sqrt(q) * delta(1), 0, sqrt(q) * delta(0), sqrt(q) * delta(1),
                    delta(1), -delta(0), -q, -delta(1), -delta(0);
                H_low = H_low * 1 / q;

                H_i.block(0, 0, 2, 3) = H_low.block(0, 0, 2, 3);
                H_i(0, 3 + i) = H_low(0, 3);
                H_i(0, 4 + i) = H_low(0, 4);
                H_i(1, 3 + i) = H_low(1, 3);
                H_i(1, 4 + i) = H_low(1, 4);
                // Kalman Gain berechnen
                K = Cov * H_i.transpose() * (H_i * Cov * H_i.transpose() + Q).inverse();
                /* std::cout << "Kalmangain: " << std::endl
                          << K << std::endl
                          << std::endl; */

                // calculate actual range bearing measurement of LM throug ICP matching of laser scan and stored LM point clouds
                Eigen::Vector2d actualMeasurement = searchLMmatches(pcl_laserscan, it.value()["points"], it.value()["pose"], convergedLandmarks);
                if (actualMeasurement(0) != -1)
                {
                    std::cout << "and converged with laser scan." << std::endl;
                    // state_vektor updaten
                    state_vector = state_vector + K * (actualMeasurement - predictedMeasurment);
                    // Cov updaten
                    Cov = (I - K * H_i) * Cov;
                }
                else
                {
                    std::cout << "but does NOT fit good enough." << std::endl;
                    // skip the update step
                }
            }
            i++;
        }
        // viewer->updatePointCloud(pcl_landmark_target, "Landmark");
        viewer->removePointCloud("Landmark");
        viewer->addPointCloud(convergedLandmarks, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Landmark");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "Landmark");
    }

    Eigen::Vector2d searchLMmatches(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan, json points, json pose, pcl::PointCloud<pcl::PointXYZ>::Ptr convergedLandmarks)
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
        transform = tfBuffer.lookupTransform("base_link", "odom", ros::Time(0), ros::Duration(1.5));
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
        if (fitnessScore < 0.0125)
        {
            Eigen::Matrix4f transformation = icp.getFinalTransformation();
            Eigen::Affine3f affineTransform;
            affineTransform.matrix() = transformation;

            pcl::PointXYZ landmarkPose(pose.at(0).get<float>(), pose.at(1).get<float>(), pose.at(2).get<float>());
            landmarkPose = pcl::transformPoint(landmarkPose, affineTransform);

            Eigen::Vector2d delta_actual;
            delta_actual(0) = landmarkPose.x - state_vector(0);
            delta_actual(1) = landmarkPose.y - state_vector(1);

            double q = delta_actual.transpose() * delta_actual;

            actualMeasurement(0) = sqrt(q);
            actualMeasurement(1) = atan2(delta(1), delta(0) - state_vector(3));

            *convergedLandmarks += *pcl_landmark_target;
        }
        else
        {
            actualMeasurement(0) = -1;
        }
        return actualMeasurement;
    }

    void publishBelieve()
    {
        //  state_vektor und Cov in Odometry Message verpacken und publishen
        tf2::Quaternion quat;
        quat.setRPY(0, 0, state_vector(2));
        geometry_msgs::Pose elipsePose;
        elipsePose.position.x = state_vector(0);
        elipsePose.position.y = state_vector(1);
        elipsePose.position.z = 0;
        elipsePose.orientation = tf2::toMsg(quat);

        // Emplementierung des Markers der Covarianz Elipse von
        // https://github.com/lrse/ros-utils/blob/master/src/poseWithCovariance_to_ellipsoid.cpp übernommen.
        // Allerdings auf meinen Use-Case (2D statt 3D) angepasst und alternative berechung der Orientierung.

        // Eigenwerte der (Positions-)Kovarianzmatrix berechnen
        Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(Cov.block(0, 0, 2, 2));
        Eigen::VectorXcd eigValues_complex = eigenSolver.eigenvalues();
        Eigen::MatrixXcd eigVectors_complex = eigenSolver.eigenvectors();
        // Check eigenvalues and eigenvectors complex part is null
        assert(eigValues_complex.imag() == Eigen::Vector2d::Zero(2));
        assert(eigVectors_complex.imag() == Eigen::Matrix2d::Zero(2, 2));
        // keep real part of the complex eigenvalues and eigenvectors
        Eigen::Vector2d eigValues = eigValues_complex.real();
        Eigen::Matrix2d eigVectors = eigVectors_complex.real();

        // Orientierung der Elipse berechnen (https://cookierobotics.com/007/)
        double theta;
        if (Cov(0, 1) == 0 && Cov(0, 0) <= Cov(1, 1))
        {
            theta = 0;
        }
        else if (Cov(0, 1) == 0 && Cov(0, 0) > Cov(1, 1))
        {
            theta = M_PI_2;
        }
        else
        {
            theta = atan2(eigValues[1] - Cov(0, 0), Cov(0, 1));
            theta = fmod(theta, 2.0 * M_PI);
            if (theta < 0.0)
            {
                theta += 2.0 * M_PI;
            }
        }

        quat.setRPY(0, 0, theta);
        elipsePose.orientation = tf2::toMsg(quat);

        // rviz Marker generieren
        visualization_msgs::Marker covEllipseMsg;
        covEllipseMsg.header.frame_id = "odom";
        covEllipseMsg.header.stamp = ros::Time::now();

        // Marker mit selbem Namespace und id überschreiben sich selber (das wollen wir)
        covEllipseMsg.ns = "covariance";
        covEllipseMsg.id = 0;

        covEllipseMsg.type = visualization_msgs::Marker::SPHERE;
        covEllipseMsg.action = visualization_msgs::Marker::ADD;

        covEllipseMsg.pose = elipsePose;
        double marker_scale = 0.5; // scale factor to allow the ellipsoid be big enough
        covEllipseMsg.scale.x = eigValues[1] * marker_scale;
        covEllipseMsg.scale.y = eigValues[0] * marker_scale;
        covEllipseMsg.scale.z = 0.01 * marker_scale;

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

    //+++++++++ Public variables ++++++++++
    tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

private:
    ros::NodeHandle n;
    ros::Publisher covEllipseMsgPublisher;
    ros::Subscriber laserScanSubscriber;
    ros::Subscriber cmdVelSubscriber;

    double last_timestamp; // Zeitpunkt als predict methode zum letzten mal aufgerufen wurde
    double v;
    double w;
    int DIMsize;
    double alpha1;
    double alpha2;
    double alpha3;
    double alpha4;
    Eigen::VectorXd state_vector;      // Zustands Vektor der Pose und der Landmarken
    Eigen::VectorXd state_vector_last; // Zustands Vektor der Pose und der Landmarken bei letztem Aufruf der Predict Methode
    Eigen::MatrixXd Cov;               // Covarianzmatrix des Zustandsvekotors (Roboterpose und Landmarken)

    std::string jsonPath;
    json stored_landmarks;
    Eigen::Vector2d delta;
    Eigen::Vector2d predictedMeasurment;
    Eigen::MatrixXd H_low;
    Eigen::MatrixXd H_i;
    Eigen::MatrixXd K;
    Eigen::Matrix2d Q;
    Eigen::MatrixXd I;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    ros::Timer viewer_timer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan;

    geometry_msgs::TransformStamped transform;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *listener;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    EKFSlam EKFSlamObject(n);

    ros::Duration(2).sleep(); // Warten bis rviz geladen ist

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();

        EKFSlamObject.predict();

        r.sleep();
    }

    return 0;
}