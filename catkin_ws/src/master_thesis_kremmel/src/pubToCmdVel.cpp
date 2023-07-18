#include "ros/ros.h"
#include <ds4_driver/Report.h>
#include <geometry_msgs/Twist.h>

class CDS4Controller
{
private:
  ros::NodeHandle n;
  geometry_msgs::Twist RobVelocities;

public:
  CDS4Controller(ros::NodeHandle& nTemp) : n(nTemp)
  {
    
  }
  ~CDS4Controller()
  {
  }
  
  void ds4Callback(const ds4_driver::Report& msg)
  {
    if (abs(msg.right_analog_y - 128.0) > 5.0 && !msg.button_l1) 
    {
      RobVelocities.linear.x = -((msg.right_analog_y - 128.0) / 750);
    }
    else
    {
      RobVelocities.linear.x = 0;
    }
    
    if (abs(msg.right_analog_x - 128.0) > 5.0 && !msg.button_r1)
    {
      RobVelocities.angular.z = -((msg.right_analog_x - 128.0) / 500);
    }
    else
    {
      RobVelocities.angular.z = 0;
    }
  }

  geometry_msgs::Twist getRobVelocities(){
    return RobVelocities;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "DS4Manager");

  ros::NodeHandle n;

  ros::Subscriber rawDS4Data;
  ros::Publisher pubVelocitiesToRobot;
  pubVelocitiesToRobot = n.advertise<geometry_msgs::Twist>("/taurob_tracker/cmd_vel_raw", 10);

  CDS4Controller ODS4Controller(n);

  rawDS4Data = n.subscribe("raw_report", 1, &CDS4Controller::ds4Callback, &ODS4Controller);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    pubVelocitiesToRobot.publish(ODS4Controller.getRobVelocities());
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
