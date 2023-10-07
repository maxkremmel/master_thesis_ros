#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include "json.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>
#include <pcl/filters/crop_box.h>

using json = nlohmann::json;

bool next_iteration;
int iterations;
json stored_landmarks;
Eigen::Matrix4d transformation_matrix;

void print4x4Matrix(const Eigen::Matrix4d &matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

void callback(const sensor_msgs::PointCloud2ConstPtr &new_laserscan_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_laserscan(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_landmark_transfored(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_landmark(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*new_laserscan_msg, *pcl_laserscan);

    double poseX = (double)stored_landmarks["landmarks"].at(1)["pose"].at(0).get<float>();
    double poseY = (double)stored_landmarks["landmarks"].at(1)["pose"].at(1).get<float>();

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(poseX-0.5, poseY-0.5, -1.5, 1.0));
    boxFilter.setMax(Eigen::Vector4f(poseX+0.5, poseY+0.5, 3.5, 1.0));
    boxFilter.setInputCloud(pcl_laserscan);
    boxFilter.filter(*pcl_laserscan);

    json pointsOfSecondLM = stored_landmarks["landmarks"].at(1)["points"];
    pcl_landmark->points.resize(pointsOfSecondLM.size());
    int i = 0;
    for (json::iterator it_points = pointsOfSecondLM.begin(); it_points != pointsOfSecondLM.end(); ++it_points)
    {
        pcl_landmark->points[i].x = it_points.value().at(0).get<float>();
        pcl_landmark->points[i].y = it_points.value().at(1).get<float>();
        pcl_landmark->points[i].z = it_points.value().at(2).get<float>();
        i++;
    }

    *pcl_landmark_transfored = *pcl_landmark;

    // The Iterative Closest Point algorithm
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(pcl_landmark);
    icp.setInputTarget(pcl_laserscan);
    icp.align(*pcl_landmark);
    icp.setMaximumIterations(1); // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in "
              << " ms" << std::endl;

    if (icp.hasConverged())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
    }
    else
    {
        PCL_ERROR("\nICP has not converged.\n");
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer("ICP demo");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // Create two vertically separated viewports
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(pcl_laserscan, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
                                                                                     (int)255 * txt_gray_lvl);
    viewer.addPointCloud(pcl_laserscan, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_in_v1");
    viewer.addPointCloud(pcl_laserscan, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_in_v2");

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(pcl_landmark_transfored, 20, 180, 20);
    viewer.addPointCloud(pcl_landmark_transfored, cloud_tr_color_h, "cloud_tr_v1", v1);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_tr_v1");

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(pcl_landmark, 180, 20, 20);
    viewer.addPointCloud(pcl_landmark, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_icp_v2");

    // Adding text descriptions in each viewport
    viewer.addText("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1920, 1080); // Visualiser window size
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

    // Display the visualiser
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            icp.align(*pcl_landmark);
            std::cout << "Applied 1 ICP iteration in "
                      << " ms" << std::endl;

            if (icp.hasConverged())
            {
                printf("\033[11A"); // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %f\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>(); // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix(transformation_matrix);                                // Print the transformation between original pose and current pose

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud(pcl_landmark, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR("\nICP has not converged.\n");
            }
        }
        next_iteration = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ICPTest");
    ros::NodeHandle n;

    ros::Subscriber laserScanSubscriber = n.subscribe<sensor_msgs::PointCloud2>("/transformed_laserscan", 10, callback);

    std::string jsonPath;

    std::string packagePath = ros::package::getPath("master_thesis_kremmel");
    jsonPath = packagePath + "/data/stored_landmarks.json";
    std::ifstream f(jsonPath);
    stored_landmarks = json::parse(f);

    next_iteration = false;
    iterations = 1;

    ros::Duration(2).sleep(); // Warten bis rviz geladen ist

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}