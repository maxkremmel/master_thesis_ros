#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Evaluator
{
public:
    Evaluator(ros::NodeHandle &nTemp) : n(nTemp)
    {
        groundTruthSubcriber.subscribe(n, "/ground_truth/state", 1);
        currentStateSubscriber.subscribe(n, "/ekf_slam/current_state", 1);
        sync.reset(new Sync(SyncRule(10), groundTruthSubcriber, currentStateSubscriber)); 
        sync->registerCallback(boost::bind(&Evaluator::callback, this, _1, _2));         // Zuweisen des synchronisierten Callbacks

        poseDiffPublisher = n.advertise<geometry_msgs::Point>("/poseDiff", 1);
        ros::Duration(3).sleep(); // Warten
    }

    void callback(const nav_msgs::Odometry::ConstPtr &groundTruthMsg, const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &currentStateMsg)
    {
        geometry_msgs::Point diff;
        double diff_x = groundTruthMsg->pose.pose.position.x - currentStateMsg->pose.pose.position.x;
        double diff_y = groundTruthMsg->pose.pose.position.y - currentStateMsg->pose.pose.position.y;
        diff.x = (float)sqrt(pow(diff_x, 2) + pow(diff_y, 2));
        poseDiffPublisher.publish(diff);
    }

private:
    ros::NodeHandle n;

    ros::Publisher poseDiffPublisher;

    message_filters::Subscriber<nav_msgs::Odometry> groundTruthSubcriber;
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> currentStateSubscriber;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::PoseWithCovarianceStamped> SyncRule;
    typedef message_filters::Synchronizer<SyncRule> Sync;
    boost::shared_ptr<Sync> sync;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "EKFSlam");
    ros::NodeHandle n;
    Evaluator Node(n);

    ros::Rate r(10); // 10 hz
    while (ros::ok)
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}