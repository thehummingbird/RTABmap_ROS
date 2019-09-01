#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/time.h"

class StopBot
{
public:
  StopBot();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
private:
  ros::NodeHandle n;
  ros::Subscriber lidar_sub;
  ros::Publisher motor_control_pub;

}; 

StopBot::StopBot()
{
  motor_control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  lidar_sub = n.subscribe("/scan",10,&StopBot::scanCallback,this);
}

void StopBot::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  geometry_msgs::Twist motor_command;
  float zero_deg = scan->ranges[360];
  if(zero_deg < 0.28)
  {
    motor_command.linear.x=0;
    motor_control_pub.publish(motor_command);
    ROS_INFO("Obstacle too close! Robot stopped!\n");
    ros::Duration(10,0).sleep();
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stop_bot");
    StopBot Obj;


    ros::spin();

    return 0;
}
